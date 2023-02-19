#if CONFIG_STM_UPDATER
#include "stm32Updater.h"
#include "main.h"

//adapted from https://github.com/mengguang/esp8266_stm32_isp


#define BLOCK_SIZE 128

#define STM32F103 0x410
#define STM32F722 0x452


char log_buffer[256];

uint8_t memory_buffer[BLOCK_SIZE];
uint8_t file_buffer[BLOCK_SIZE];

#define DEBUG_PRINT(...) \
	do { \
		snprintf(log_buffer, sizeof(log_buffer), __VA_ARGS__); \
  		debug_log(); \
	}while(0);


void reset_stm32_to_isp_mode()
{
	// reset STM32 into DFU mode
	pinMode(RESET_PIN, OUTPUT);
	pinMode(BOOT0_PIN, OUTPUT);
	digitalWrite(BOOT0_PIN, HIGH);
	delay(20);
	digitalWrite(RESET_PIN, LOW);
	delay(100);
	digitalWrite(RESET_PIN, HIGH);
	delay(100);
}

void reset_stm32_to_app_mode()
{
	DEBUG_PRINT("Reset to APP mode...");
	pinMode(RESET_PIN, OUTPUT);
	pinMode(BOOT0_PIN, OUTPUT);
	digitalWrite(BOOT0_PIN, LOW);
	delay(10);
	digitalWrite(RESET_PIN, LOW);
	delay(50);
	digitalWrite(RESET_PIN, HIGH);
}

void stm32flasher_hardware_init()
{
	Serial.begin(115200, SERIAL_8E1);
	Serial.setTimeout(500);
}

void debug_log()
{
	websocket_send_txt(log_buffer);
}

static inline void isp_serial_write(uint8_t const * buffer, uint8_t const length)
{
	Serial.write(buffer, length);
	//Serial.flush();
}

static inline void isp_serial_write(uint8_t const buffer)
{
	isp_serial_write(&buffer, 1);
}

static inline uint8_t isp_serial_read(uint8_t *buffer, uint8_t const length, uint8_t const timeout = 100)
{
	Serial.setTimeout(timeout);
	return Serial.readBytes(buffer, length);
}

static inline void isp_serial_flush_rx(void)
{
	while (Serial.available())
		Serial.read();
}


enum {
	RESP_ACK = 0x79,
	RESP_NACK = 0x1F,
	RESP_BUSY = 0x76,
};

int8_t wait_for_ack(char const *when, int8_t timeout = 20)
{
	uint8_t cmd = 0;
	uint8_t nread;
	while (timeout--) {
		nread = isp_serial_read(&cmd, 1);
		if (cmd == RESP_ACK) {
			return 1;
		} else if (cmd == RESP_NACK) {
			DEBUG_PRINT("got NACK when: %s", when);
			break;
		} else if (cmd == RESP_BUSY) {
			DEBUG_PRINT("got BUSY when: %s", when);
		} else if (nread) {
			DEBUG_PRINT("[WARNING] got unknown response: %d when: %s.", cmd, when);
		} else {
			delay(1);
		}
	}

	if (timeout <= 0) {
		DEBUG_PRINT("[ERROR] no response when: %s.", when);
	}

	return -1;
}

int8_t init_chip(void)
{
	uint8_t cmd = 0x7F;

#define STM_MODE_LOOPS 3
#define STM_INIT_LOOPS 5

	for (int j = 0; j < STM_MODE_LOOPS; j++) {
		DEBUG_PRINT("Trying to init chip... %u / %u", j+1, STM_MODE_LOOPS);
		reset_stm32_to_isp_mode();
		for (int i = 0; i < STM_INIT_LOOPS; i++) {
			isp_serial_flush_rx();
			isp_serial_write(&cmd, 1);
			if (0 <= wait_for_ack("init_chip_write_cmd", 20)) {
				DEBUG_PRINT("init chip succeeded.");
				return 0;
			}
		}
	}
	DEBUG_PRINT("[ERROR] init chip failed.");
	return -1;
}

uint8_t cmd_generic(uint8_t const command)
{
	uint8_t cmd[2];
	cmd[0] = command;
	cmd[1] = command ^ 0xFF;
	isp_serial_flush_rx();
	isp_serial_write(cmd, sizeof(cmd));
	return (0 <= wait_for_ack("cmd_generic"));
}

uint8_t cmd_get()
{
	uint8_t bootloader_ver = 0;
	if (cmd_generic(0x00))
	{
		uint8_t * buffer = (uint8_t*)malloc(257);
		if (!buffer) return 0;
		isp_serial_read(buffer, 2);
		uint8_t len = buffer[0];
		bootloader_ver = buffer[1];
		isp_serial_read(buffer, len);
		free(buffer);
		if (wait_for_ack("cmd_get") != 1)
			return 0;

		DEBUG_PRINT("Bootloader version: 0x%02X", bootloader_ver);

		if (bootloader_ver < 20 || bootloader_ver >= 100) {
			DEBUG_PRINT("[ERROR] Invalid bootloader");
			bootloader_ver = 0;
		}
	}
	return bootloader_ver;
}

int8_t cmd_getID(void)
{
	int8_t retval = -1;
	if (cmd_generic(0x02)) {
		uint8_t buffer[3] = {0, 0, 0};
		isp_serial_read(buffer, 3);

		uint16_t id = buffer[1];
		id <<= 8;
		id += buffer[2];

		retval = wait_for_ack("cmd_getID");
		switch (id) {
			case STM32F103:
				DEBUG_PRINT("STM32F1 found!");
				break;
			case STM32F722:
				DEBUG_PRINT("STM32F7 found!");
				break;
			default:
				DEBUG_PRINT("[ERROR] Not supported Chip ID (0x%X).", id);
				retval = 0;
				break;
		}
	}
	return retval;
}

void encode_address_and_send(uint32_t const address)
{
	uint8_t const b3 = (uint8_t)((address >> 0) & 0xFF);
	uint8_t const b2 = (uint8_t)((address >> 8) & 0xFF);
	uint8_t const b1 = (uint8_t)((address >> 16) & 0xFF);
	uint8_t const b0 = (uint8_t)((address >> 24) & 0xFF);

	uint8_t crc = (uint8_t)(b0 ^ b1 ^ b2 ^ b3);
	uint8_t enc_address[5] = {b0, b1, b2, b3, crc};
	isp_serial_write(enc_address, sizeof(enc_address));
}

uint8_t cmd_read_memory(uint32_t const address, uint8_t const length)
{
	if (cmd_generic(0x11))
	{
		encode_address_and_send(address);
		if (wait_for_ack("write_address") != 1)
		{
			return 0;
		}
		uint8_t nread = length - 1;
		uint8_t crc = nread ^ 0xFF;
		uint8_t buffer[2];
		buffer[0] = nread;
		buffer[1] = crc;
		isp_serial_write(buffer, 2);
		if (wait_for_ack("write_address") != 1)
		{
			return 0;
		}
		uint8_t nreaded = 0;
		while (nreaded < length)
		{
			nreaded += isp_serial_read(memory_buffer + nreaded, length - nreaded);
		}
		return 1;
	}
	return 0;
}

uint8_t cmd_write_memory(uint32_t const address, uint8_t const length)
{
	if (cmd_generic(0x31))
	{
		encode_address_and_send(address);
		if (wait_for_ack("write_address") != 1)
		{
			return 0;
		}
		uint8_t nwrite = length - 1;
		uint8_t buffer[1];
		buffer[0] = nwrite;
		isp_serial_write(buffer, 1);
		uint8_t crc = nwrite;
		for (int i = 0; i < length; i++)
		{
			crc = crc ^ memory_buffer[i];
		}
		isp_serial_write(memory_buffer, length);
		buffer[0] = crc;
		isp_serial_write(buffer, 1);
		return wait_for_ack("write_data");
	}
	return 0;
}

// if bootversion < 0x30
// pg 7 of https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf
static int8_t cmd_erase_memory(uint32_t const pages, uint8_t page_offset)
{
	if (cmd_generic(0x43)) {
#if 0 //(FLASH_OFFSET == 0)
		DEBUG_PRINT("erasing all memory...");
		// Global erase aka mass erase
		uint8_t cmd[2];
		cmd[0] = 0xFF;
		cmd[1] = 0x00;
		isp_serial_write(cmd, sizeof(cmd));
		return wait_for_ack("mass_erase");
#else
		uint8_t checksum = 0, page;
		isp_serial_write((uint8_t)(pages-1));
		checksum ^= (pages-1);
		for (page = page_offset; page < (page_offset + pages); page++) {
			isp_serial_write(page);
			checksum ^= page;
		}
		isp_serial_write(checksum);
		return wait_for_ack("erase_pages");
#endif
	}
	return -1;
}

// else if bootversion >= 0x30
static int8_t cmd_erase_memory_extended(uint32_t const pages, uint8_t page_offset)
{
	if (cmd_generic(0x44)) {
#if 0 //(FLASH_OFFSET == 0)
		uint8_t cmd[3];
		cmd[0] = 0xFF;
		cmd[1] = 0xFF;
		cmd[2] = 0x00;
		isp_serial_write(cmd, sizeof(cmd));
		return wait_for_ack("mass_erase", 100);
#else
		uint8_t checksum = 0, page;
		// Send number of pages (16bit, MSB first)
		isp_serial_write(0);
		isp_serial_write((uint8_t)(pages-1));
		checksum ^= (pages-1);
		for (page = page_offset; page < (page_offset + pages); page++) {
			// Send page number (16bit, MSB first)
			isp_serial_write(0);
			isp_serial_write(page);
			checksum ^= page;
		}
		isp_serial_write(checksum);
		return wait_for_ack("erase_pages", 100);
#endif
	}
	return -1;
}

int8_t cmd_erase(uint32_t const start_addr, uint32_t const filesize, uint8_t const bootloader_ver)
{
	// Erase only defined pages
	uint8_t pages = (FLASH_SIZE / FLASH_PAGE_SIZE);
	if (filesize)
		pages = (filesize + (FLASH_PAGE_SIZE - 1)) / FLASH_PAGE_SIZE;
	uint8_t page_offset = ((start_addr - FLASH_START) / FLASH_PAGE_SIZE);
	DEBUG_PRINT("erasing pages: %u...%u", page_offset, (page_offset + pages));

	if (bootloader_ver < 0x30)
		return cmd_erase_memory(pages, page_offset);
	return cmd_erase_memory_extended(pages, page_offset);
}

uint8_t cmd_go(uint32_t const address)
{
	if (cmd_generic(0x21)) {
		encode_address_and_send(address);
		return wait_for_ack("cmd_go");
	}
	return 0;
}

uint8_t esp8266_spifs_write_file(const char *filename, uint32_t begin_addr)
{
	if (!FILESYSTEM.exists(filename))
	{
		DEBUG_PRINT("[ERROR] file: %s doesn't exist!", filename);
		return 0;
	}
	File fp = FILESYSTEM.open(filename, "r");
	uint32_t filesize = fp.size();
	DEBUG_PRINT("filesize: %d", filesize);
	if ((FLASH_SIZE - FLASH_OFFSET) < filesize) {
		DEBUG_PRINT("[ERROR] file is too big!");
		fp.close();
		return 0;
	}

	if (begin_addr < FLASH_START)
		begin_addr += FLASH_START;
    String message = "Using flash base: 0x";
    message += String(begin_addr, 16);
    websocket_send_txt(message);

	stm32flasher_hardware_init();

	if (init_chip() < 0) {
		fp.close();
		return 0;
	}

	uint8_t const bootloader_ver = cmd_get();
	if (bootloader_ver == 0) {
		fp.close();
		return 0;
	}

	if (cmd_getID() < 0) {
		fp.close();
		return 0;
	}

	if (cmd_erase(begin_addr, filesize, bootloader_ver) < 0) {
		DEBUG_PRINT("[ERROR] erase Failed!");
		fp.close();
		return 0;
	}
	DEBUG_PRINT("erase Success!");

	DEBUG_PRINT("begin to write file.");
	uint32_t junks = (filesize + (BLOCK_SIZE - 1)) / BLOCK_SIZE;
	uint32_t junk = 0;
	uint8_t proc;
	while (fp.position() < filesize) {
		uint8_t nread = BLOCK_SIZE;
		if ((filesize - fp.position()) < BLOCK_SIZE) {
			nread = filesize - fp.position();
		}
		nread = fp.readBytes((char *)memory_buffer, nread);

		//DEBUG_PRINT("write bytes: %d, file position: %d", nread, fp.position());
		proc = (++junk * 100) / junks;
		if ((junk % 40) == 0 || junk >= junks)
			DEBUG_PRINT("Write %u%%", proc);

		uint8_t result = cmd_write_memory(begin_addr + fp.position() - nread, nread);
		if (result != 1) {
			DEBUG_PRINT("[ERROR] file write failed.");
			fp.close();
			return 0;
		}
	}
	DEBUG_PRINT("file write succeeded.");

	DEBUG_PRINT("begin to verify file.");
	fp.seek(0, SeekSet);
	junk = 0;
	while (fp.position() < filesize) {
		uint8_t nread = BLOCK_SIZE;
		if ((filesize - fp.position()) < BLOCK_SIZE)
		{
			nread = filesize - fp.position();
		}
		nread = fp.readBytes((char *)file_buffer, nread);

		//DEBUG_PRINT("read bytes: %d, file position: %d", nread, fp.position());
		proc = (++junk * 100) / junks;
		if ((junk % 40) == 0 || junk >= junks)
			DEBUG_PRINT("Verify %u%%", proc);

		uint8_t result = cmd_read_memory(begin_addr + fp.position() - nread, nread);
		if (result != 1) {
			DEBUG_PRINT("[ERROR] read memory failed: %d", fp.position());
			fp.close();
			return 0;
		}
		result = memcmp(file_buffer, memory_buffer, nread);
		if (result != 0) {
			DEBUG_PRINT("[ERROR] verify failed.");
			fp.close();
			return 0;
		}
	}

	fp.close();
	DEBUG_PRINT("verify file succeeded.");
	DEBUG_PRINT("start application.");
	//reset_stm32_to_app_mode();
	cmd_go(FLASH_START);
	return 1;
}
#endif // CONFIG_STM_UPDATER
