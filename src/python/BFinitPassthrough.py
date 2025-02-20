import serial, time, sys, re
import argparse
import serials_find
import SerialHelper
import bootloader
from query_yes_no import query_yes_no
from console_log import *


SCRIPT_DEBUG = 0


class PassthroughEnabled(Exception):
    pass

class PassthroughFailed(Exception):
    pass


def _validate_serialrx(rl, config, expected):
    found = False
    if type(expected) == str:
        expected = [expected]
    rl.set_delimiters(["# "])
    rl.clear()
    rl.write_str("get serialrx_%s" % config)
    line = rl.read_line(1.).strip()
    for key in expected:
        key = " = %s" % key
        if key in line:
            found = True
            break
    return found


def bf_passthrough_init(port, requestedBaudrate, half_duplex=False):
    set_debug(SCRIPT_DEBUG)

    sys.stdout.flush()
    print_header("======== PASSTHROUGH INIT ========")
    print_log("  Trying to initialize %s @ %s" % (port, requestedBaudrate,))

    s = serial.Serial(port=port, baudrate=115200,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)

    rl = SerialHelper.SerialHelper(s, 3., ["# "])
    rl.clear()
    # Send start command '#'
    rl.write_str("#", half_duplex)
    start = rl.read_line(2.).strip()
    #print_debug("BF INIT: '%s'" % start.replace("\r", ""))
    if not start or not start.endswith("#"):
        raise PassthroughEnabled("No CLI available. Already in passthrough mode?, If this fails reboot FC and try again!")

    serial_check = []
    expected_provider = [["CRSF", "ELRS"], "GHST"][half_duplex]
    if not _validate_serialrx(rl, "provider", expected_provider):
        serial_check.append("serialrx_provider != %s" % expected_provider)
    if not _validate_serialrx(rl, "inverted", "OFF"):
        serial_check.append("serialrx_inverted != OFF")
    if not _validate_serialrx(rl, "halfduplex", ["OFF", "AUTO"]):
        serial_check.append("serialrx_halfduplex != OFF/AUTO")

    if serial_check:
        error = "\n\n [ERROR] Invalid serial RX configuration detected:\n"
        for err in serial_check:
            error += "    !!! %s !!!\n" % err
        error += "\n    Please change the configuration and try again!\n"
        raise PassthroughFailed(error)

    SerialRXindex = ""

    print_log("\nAttempting to detect FC UART configuration...")

    rl.set_delimiters(["\n"])
    rl.clear()
    rl.write_str("serial")

    while True:
        line = rl.read_line().strip()
        #print("FC: '%s'" % line)
        if not line or "#" in line:
            break

        if line.startswith("serial"):
            print_debug("  '%s'" % line)
            config = re.search('serial ([0-9]+) ([0-9]+) ', line)
            if config and config.group(2) == "64":
                print_log("    ** Serial RX config detected: '%s'" % line)
                SerialRXindex = config.group(1)
                if not SCRIPT_DEBUG:
                    break
    rl.clear()

    if not SerialRXindex:
        raise PassthroughFailed("!!! RX Serial not found !!!!\n  Check configuration and try again...")

    if requestedBaudrate is None:
        requestedBaudrate = 0
    cmd = "serialpassthrough %s %s" % (SerialRXindex, requestedBaudrate, )

    print_log("Enabling serial passthrough...")
    print_log("  CMD: '%s'" % cmd)
    rl.write_str(cmd)
    #time.sleep(.2)

    # Read configured baudrate
    baud = None
    dbg = rl.read_line(.5)
    while dbg:
        dbg = dbg.strip()
        print_log("%s" % dbg)
        if "baud =" in dbg:
            baud = dbg.split("baud =")[1].strip()
        dbg = rl.read_line(.5)

    s.close()
    print_log("======== PASSTHROUGH DONE ========")
    # Let the CRSFv3 to fallback to 420k baud
    time.sleep(1.5)
    try:
        return int(eval(baud))
    except:
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Initialize BetaFlight passthrough and optionally send a reboot comamnd sequence")
    parser.add_argument("-b", "--baud", type=int, default=0,
        help="Baud rate for passthrough communication")
    parser.add_argument("-p", "--port", type=str,
        help="Override serial port autodetection and use PORT")
    parser.add_argument("-r", "--target", type=str,
        help="The target firmware that is going to be uploaded")
    parser.add_argument("-nr", "--no-reset", action="store_false",
        dest="reset_to_bl", help="Do not send reset_to_bootloader command sequence")
    parser.add_argument("-hd", "--half-duplex", action="store_true",
        dest="half_duplex", help="Use half duplex mode")
    parser.add_argument("-t", "--type", type=str, default="ESP82",
        help="Defines flash target type which is sent to target in reboot command")
    args = parser.parse_args()

    if (args.port == None):
        args.port = serials_find.get_serial_port()

    try:
        bf_passthrough_init(args.port, args.baud)
    except PassthroughEnabled as err:
        print_warning(str(err))

    if args.reset_to_bl:
        boot_cmd = bootloader.get_init_seq(['CRSF', 'GHST'][args.half_duplex], args.type)
        try:
            bootloader.reset_to_bootloader(args.port, args.baud, boot_cmd, args.target)
        except bootloader.WrongTargetSelected as err:
            print_error(str(err))
            exit(-1)
