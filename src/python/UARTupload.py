import serial
from xmodem import XMODEM
import time
import sys
import logging
import os
import serials_find

logging.basicConfig(level=logging.ERROR)

filename = ''
filesize = 0
filechunks = 0

BootloaderInitSeq1 = bytes([0xEC,0x04,0x32,0x62,0x6c,0x0A])
BootloaderInitSeq2 = bytes([0x62,0x62,0x62,0x62,0x62,0x62])

sys.stdout.flush()

def dbg_print(line):
    sys.stdout.write(line)
    sys.stdout.flush()

try:
    filename = sys.argv[1]
except:
    dbg_print("Filename not provided, going to use default firmware.bin")
    filename = "firmware.bin"

if not os.path.exists(filename):
    msg = "[FAILED] bin file '%s' does not exist\n" % filename
    dbg_print(msg)
    raise EnvironmentError(msg)

port = serials_find.get_serial_port()
dbg_print("Going to use %s\n" % port)
s = serial.Serial(port=port, baudrate=420000, bytesize=8, parity='N', stopbits=1, timeout=.5, xonxoff=0, rtscts=0)
s.timeout = 4.

try:
    already_in_bl = s.read(3).decode('utf-8')
except UnicodeDecodeError:
    already_in_bl = ""

if 'CC' not in already_in_bl:
    s.timeout = .5
    s.write_timeout = .5

    currAttempt = 0
    gotBootloader = False

    dbg_print("\nAttempting to reboot into bootloader...\n")

    while gotBootloader == False:

        currAttempt += 1
        dbg_print("[%2u] retry...\n" % currAttempt)
        time.sleep(.5)
        if 10 < currAttempt:
            msg = "Failed to get to BL in reasonable time\n"
            dbg_print(msg)
            raise EnvironmentError(msg)

        # request reboot
        s.write(BootloaderInitSeq1)
        s.flush()
        start = time.time()
        while ((time.time() - start) < 2):
            try:
                line = s.readline().decode('utf-8')
                if not line and s.in_waiting:
                    line = s.read(128).decode('utf-8')
            except UnicodeDecodeError:
                continue
            #if line:
            #    dbg_print("line : '%s'\n" % (line.strip(), ))
            #if "'2bl', 'bbb'" in line or "ExpressLRS" in line:
            if "Bootloader for ExpressLRS" in line:
                for idx in range(2):
                    line = s.readline().decode('utf-8')
                    if "BL_TYPE" in line:
                        # do check...
                        dbg_print("line : '%s'\n" % (line, ))
                        bl_ver = line.strip()[8:].strip()
                        dbg_print("Bootloader type : '%s'\n" % (bl_ver, ))
                        break
                #time.sleep(.5)
                # notify bootloader to start uploading
                s.write(BootloaderInitSeq2)
                s.flush()
                dbg_print("Got into bootloader after: %u attempts\n" % (currAttempt))
                gotBootloader = True
                break

    # change timeout to 30sec
    s.timeout = 30.
    s.write_timeout = 5.

    # sanity check! Make sure the bootloader is started
    dbg_print("Wait sync...")
    start = time.time()
    while True:
        char = s.read(3).decode('utf-8')
        if char == 'CCC':
            break
        #if char:
        #    dbg_print("char : '%s'\n" % (char.strip(), ))
        if ((time.time() - start) > 15):
            msg = "\n[FAILED] Unable to communicate with bootloader...\n"
            dbg_print(msg)
            raise EnvironmentError(msg)
    dbg_print("  ... sync OK\n")
else:
    dbg_print("\nWe were already in bootloader\n")

# change timeout to 5sec
s.timeout = 5.
s.write_timeout = 5.

# open binary
stream = open(filename, 'rb')
filesize = os.stat(filename).st_size
filechunks = filesize/128

dbg_print("uploading %d bytes...\n" % (filesize,))

def StatusCallback(total_packets, success_count, error_count):
    sys.stdout.flush()
    if (total_packets % 10 == 0):
        if (error_count > 0):
            dbg_print(str(round((total_packets/filechunks)*100)) + "% err: " + str(error_count) + "\n")
        else:
            dbg_print(str(round((total_packets/filechunks)*100)) + "%\n")

def getc(size, timeout=3):
    return s.read(size) or None

def putc(data, timeout=3):
    return s.write(data)

modem = XMODEM(getc, putc, mode='xmodem')
#modem.log.setLevel(logging.DEBUG)
status = modem.send(stream, retry=10, callback=StatusCallback)

s.close()
stream.close()

if (status):
    dbg_print("Success!!!!\n\n")
else:
    dbg_print("[FAILED] Upload failed!\n\n")
    raise EnvironmentError('Failed to Upload')
