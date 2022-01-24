import serial
import time
import sys
import logging
import os
import re
import BFinitPassthrough
import SerialHelper
import bootloader
from esptool import main as esptool_main


SCRIPT_DEBUG = 0


def dbg_print(line=''):
    sys.stdout.write(line)
    sys.stdout.flush()
    return


def uart_upload(port, filename, baudrate, key=None, target="", skip_reset=False, esp32=False):
    dbg_print("=================== FIRMWARE UPLOAD ===================\n")
    dbg_print("  Bin file '%s'\n" % filename)
    dbg_print("  Port %s @ %s\n" % (port, baudrate))

    logging.basicConfig(level=logging.ERROR)

    cmd_reboot_to_bootloader = bootloader.get_init_seq('CRSF', key)
    dbg_print("  Using CRSF (full duplex)!\n")

    if not os.path.exists(filename):
        msg = "[FAILED] file '%s' does not exist\n" % filename
        dbg_print(msg)
        raise SystemExit(msg)

    detected_baud = baudrate

    # Init Betaflight passthrough
    try:
        detected_baud = BFinitPassthrough.bf_passthrough_init(port, baudrate)
        if detected_baud is None:
            detected_baud = baudrate
    except BFinitPassthrough.PassthroughEnabled as info:
        dbg_print("  Warning: '%s'\n" % info)
    except BFinitPassthrough.PassthroughFailed as failed:
        raise

    # Reset into bootloader
    if not skip_reset:
        dbg_print("======== RESET TO BOOTLOADER ========")
        s = serial.Serial(port=port, baudrate=detected_baud,
            bytesize=8, parity='N', stopbits=1,
            timeout=1, xonxoff=0, rtscts=0)
        rl = SerialHelper.SerialHelper(s, 3.)
        rl.clear()
        # Let the CRSFv3 to fallback to 420k baud
        time.sleep(1.5)
        rl.write(cmd_reboot_to_bootloader)
        s.flush()
        rx_target = rl.read_line().strip()
        dbg_print("  Receiver reported '%s' target" % rx_target)
        '''
        flash_target = re.sub("_VIA_.*", "", args.target.upper())
        if rx_target == "":
            dbg_print("Cannot detect RX target, blindly flashing!")
        elif rx_target != flash_target:
            if query_yes_no("\n\n\nWrong target selected! your RX is '%s', trying to flash '%s', continue? Y/N\n" % (rx_target, flash_target)):
                dbg_print("Ok, flashing anyway!")
            else:
                raise WrongTargetSelected("Wrong target selected your RX is '%s', trying to flash '%s'" % (rx_target, flash_target))
        elif flash_target != "":
            dbg_print("Verified RX target '%s'" % (flash_target))
        '''
        time.sleep(.5)
        s.close()

    # Prepare to upload
    args = [
        "-b", str(detected_baud),
        "-p", port,
        "-c", "esp8266",
        "--before", "no_reset",
        "--after", "soft_reset",
        "write_flash", "0x0", os.path.abspath(filename)
    ]
    cmd = " ".join(args)
    dbg_print("======== UPLOADING ========")
    dbg_print("  Using command: '%s'" % cmd)

    esptool_main(args)
