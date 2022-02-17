import serial
import time
import sys
import logging
import os
import re
import bootloader
import BFinitPassthrough
import SerialHelper
from console_log import *
from xmodem import XMODEM


SCRIPT_DEBUG = 0


def uart_upload(port, filename, baudrate, ghst=False, key=None, target=""):
    half_duplex = ghst
    ignore_incorrect_target = False

    set_debug(SCRIPT_DEBUG)

    print_header("=================== FIRMWARE UPLOAD ===================")
    print_log("  Bin file '%s'" % filename)
    print_log("  Port %s @ %s" % (port, baudrate))

    # Needed for XMODEM logging
    logging.basicConfig(level=logging.ERROR)

    cmd_reboot_to_bootloader = bootloader.get_init_seq(['CRSF', 'GHST'][ghst], key)
    cmd_trigger_upload = bootloader.get_bootloader_trigger_seq()

    if not os.path.exists(filename):
        msg = "[FAILED] file '%s' does not exist" % filename
        # print_error(msg)
        raise SystemExit(msg)

    detected_baud = baudrate

    # Init Betaflight passthrough
    try:
        detected_baud = BFinitPassthrough.bf_passthrough_init(port, None, half_duplex)
        if detected_baud is None:
            detected_baud = baudrate
    except BFinitPassthrough.PassthroughEnabled as info:
        print_warning("  Warning: '%s'" % info)
    except BFinitPassthrough.PassthroughFailed as failed:
        raise SystemExit(failed)

    # Prepare to upload
    s = serial.Serial(port=port, baudrate=detected_baud,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)
    rl = SerialHelper.SerialHelper(s, 2., ["CCC"], half_duplex)
    rl.clear()

    gotBootloader = False
    if bootloader.BAUDRATE_STM_BOOTLOADER == detected_baud:
        # Check again if we're in the bootloader now that passthrough is setup;
        #   Note: This is for button-method flashing
        gotBootloader = 'CCC' in rl.read_line()

    # Init bootloader
    if not gotBootloader:
        print_header("======== RESET TO BOOTLOADER ========")
        # legacy bootloader requires a 500ms delay
        delay_seq2 = .5
        currAttempt = 0

        # rl.set_timeout(2.)
        rl.set_delimiters(["\n", "CCC"])

        while gotBootloader == False:
            currAttempt += 1
            if 10 < currAttempt:
                msg = "[FAILED] to get to BL in reasonable time"
                #print_error(msg)
                raise SystemExit(msg)

            if 5 <= currAttempt:
                # Enable debug logs after 5 retries
                set_debug(True)

            print_log("  [%1u] retry... (baud: %s)" % (currAttempt, s.baudrate,))

            # Change baudrate to match ELRS firmware
            s.baudrate = detected_baud
            # clear RX buffer before continuing
            rl.clear()
            # request reboot
            rl.write(cmd_reboot_to_bootloader)

            if 2 == currAttempt:
                # BL not detected, change back to given baudrate
                detected_baud = baudrate
            elif 4 == currAttempt:
                # BL not detected, change back to default
                detected_baud = bootloader.BAUDRATE_STM_BOOTLOADER

            start = time.time()
            while ((time.time() - start) < 2.):
                line = rl.read_line().strip()
                if not line:
                    # timeout
                    continue

                print_debug(" **DBG : '%s'" % line)

                if "BL_TYPE" in line:
                    bl_type = line[8:].strip()
                    print_log("    Bootloader type found : '%s'" % bl_type)
                    # Newer bootloaders do not require delay but keep
                    # a 100ms just in case
                    delay_seq2 = .1
                    continue

                versionMatch = re.search('=== (v.*) ===', line, re.IGNORECASE)
                if versionMatch:
                    bl_version = versionMatch.group(1)
                    print_log("    Bootloader version found : '%s'" % bl_version)

                elif "hold down button" in line:
                    time.sleep(delay_seq2)
                    rl.write(cmd_trigger_upload)
                    gotBootloader = True
                    break

                elif "CCC" in line:
                    # Button method has been used so we're not catching the header;
                    #  let's jump right to the sanity check if we're getting enough C's
                    gotBootloader = True
                    break

                elif line.endswith("_RX") and not ignore_incorrect_target:
                    bootloader.validate_reported_rx_type(line, target)
                    ignore_incorrect_target = True
                    # change baurate to communicate with the bootloader
                    s.baudrate = bootloader.BAUDRATE_STM_BOOTLOADER

        print_log("  Got into bootloader after: %u attempts" % currAttempt)

        # sanity check! Make sure the bootloader is started
        print_log("  Wait sync... ", nl=False)
        rl.set_delimiters(["CCC"])
        if "CCC" not in rl.read_line(15.):
            print_error("FAILED!")
            msg = "[FAILED] Unable to communicate with bootloader..."
            raise SystemExit(msg)
        print_log("OK")

    print_header("======== UPLOADING ========")
    # change timeout to 5sec
    s.timeout = 5.
    s.write_timeout = .3

    # open binary
    stream = open(filename, 'rb')
    filesize = os.stat(filename).st_size
    filechunks = filesize / 128

    print_log("  uploading %d bytes..." % filesize)

    def StatusCallback(total_packets, success_count, error_count):
        #sys.stdout.flush()
        if (total_packets % 10 == 0) or filechunks <= total_packets:
            dbg = "    " + str(round((total_packets / filechunks) * 100)) + "%"
            if (error_count > 0):
                dbg += ", err: " + str(error_count)
            print_log(dbg)

    def getc(size, timeout=3):
        return s.read(size) or None

    def putc(data, timeout=3):
        cnt = s.write(data)
        if half_duplex:
            s.flush()
            # Clean RX buffer in case of half duplex
            #   All written data is read into RX buffer
            s.read(cnt)
        return cnt

    s.reset_input_buffer()

    modem = XMODEM(getc, putc, mode='xmodem')
    #modem.log.setLevel(logging.DEBUG)
    status = modem.send(stream, retry=10, callback=StatusCallback)

    s.close()
    stream.close()

    if (status):
        print_info("  Success!!!!\n")
    else:
        #print_error("[FAILED] Upload failed!")
        raise SystemExit('Failed to Upload')
