import serial
import time
import os
import BFinitPassthrough
import bootloader
from console_log import *
import esptool


SCRIPT_DEBUG = 0


def esp_read_chip_id(port, baud, retries=2):
    print_log("  *** Trying to detect bootloader ***")
    try:
        esp = esptool.ESPLoader.detect_chip(port, baud, "no_reset", False, retries)
        chip_is = esp.get_chip_description()
        print(f"Detected chip is: {chip_is}")
        if esp.IS_STUB:
            esp.soft_reset(True) # Stay in ROM loader
        return chip_is, esp
    except (esptool.FatalError, esptool.UnsupportedCommandError, OSError) as error:
        print_log(error)
        print_log("      Not detected")
        return "", None


def send_training_to_esp_rom(port, baud):
    s = serial.Serial(port=port, baudrate=baud,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)
    # this is the training sequ for the ROM bootloader,
    #   we send it here so it doesn't auto-neg to the wrong baudrate
    #   by the BootloaderInitSeq that we send to reset ELRS
    s.write(b'\x07\x07\x12\x20' + 32 * b'\x55')
    s.flush()
    time.sleep(0.2)
    s.close()


def uart_upload(port, filename, baudrate, key=None, target=""):
    print_header("=================== FIRMWARE UPLOAD ===================")
    print_log("  Bin file '%s'" % filename)
    print_log("  Port %s @ %s" % (port, baudrate))

    cmd_reboot_to_bootloader = bootloader.get_init_seq('CRSF', key)

    if not os.path.exists(filename):
        msg = "[FAILED] file '%s' does not exist" % filename
        raise SystemExit(msg)

    detected_baud = baudrate

    chip_type, esp = esp_read_chip_id(port, detected_baud)
    if not chip_type:
        # Init Betaflight passthrough
        try:
            detected_baud = BFinitPassthrough.bf_passthrough_init(port, None)
            if detected_baud is None:
                detected_baud = baudrate
        except BFinitPassthrough.PassthroughEnabled as info:
            print_warning("  Warning: '%s'" % info)
        except BFinitPassthrough.PassthroughFailed as failed:
            raise SystemExit(failed)
        # Reset into bootloader
        bootloader.reset_to_bootloader(
            port, detected_baud, cmd_reboot_to_bootloader, target)
        # send_training_to_esp_rom(port, detected_baud)
        chip_type, esp = esp_read_chip_id(port, detected_baud)
        if not esp:
            raise SystemExit("Cannot connect to ESP")

    # Prepare to upload
    print_header("======== UPLOADING ========")
    if esp:
        _binary = open(os.path.abspath(filename), 'rb')
        class TempArgs():
            compress = None
            no_compress = True
            no_stub = False
            encrypt = False
            flash_size = "keep"
            flash_mode = "keep"
            flash_freq = "keep"
            addr_filename = []
            erase_all = False
            verify = False
        args = TempArgs()
        args.addr_filename.append((0, _binary))
        try:
            esptool.write_flash(esp, args)
            # flash_finish will trigger a soft reset
            esp.soft_reset(False)
        finally:
            _binary.close()
            esp._port.close()
        return
    # --------------------
    chip_type = ["auto", "esp8266"][chip_type.startswith("ESP82")]
    args = [
        "-b", str(detected_baud),
        "-p", port,
        "-c", chip_type,
        "--before", "no_reset",
        "--after", "soft_reset",
        "write_flash", "0x0", os.path.abspath(filename)
    ]
    cmd = " ".join(args)
    print_log("  Using command: '%s'\n" % cmd)
    esptool.main(args)
