import sys
from console_log import *


BAUDRATE_STM_BOOTLOADER = 420000
BAUDRATE_CRSF = 420000
BAUDRATE_ELRS_PROTO = 1000000

ELRS_BOOT_CMD_DEST = ord('b')
ELRS_BOOT_CMD_ORIG = ord('l')

INIT_SEQ = {
    "CRSF": [0xEC,0x04,0x32,ELRS_BOOT_CMD_DEST,ELRS_BOOT_CMD_ORIG],
    "GHST": [0x89,0x04,0x32,ELRS_BOOT_CMD_DEST,ELRS_BOOT_CMD_ORIG],
}

def calc_crc8(payload, poly=0xD5):
    crc = 0
    for data in payload:
        crc ^= data
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
    return crc & 0xFF


def get_init_seq(module, key=None):
    print_log("  Init reboot command for %s" % module)
    payload = list(INIT_SEQ.get(module, []))
    if payload:
        if key:
            if type(key) == str:
                key = [ord(x) for x in key]
            payload += key
            payload[1] += len(key)
        payload += [calc_crc8(payload[2:])]
    return bytes(payload)


def get_bootloader_trigger_seq():
    return bytes([ord('b')] * 6)


class WrongTargetSelected(Exception):
    pass


def dbg_print(line=''):
    sys.stdout.write(line + '\n')
    sys.stdout.flush()


def validate_reported_rx_type(rx_target, target):
    from query_yes_no import query_yes_no
    if rx_target:
        print_log("  Receiver reported target: '%s'" % rx_target)
        if rx_target.endswith("_RX"):
            # remove suffix
            rx_target = rx_target[:-3].upper()
    if rx_target == "":
        print_warning("  Cannot detect RX target, blindly flashing!")
    elif target == "":
        print_warning("  Unable to verify without target, blindly flashing!")
    elif rx_target != target:
        message = "  Wrong target selected! your RX is '%s', trying to flash '%s'" % (rx_target, target)
        if query_yes_no(f"\n\n\n{message}, continue? Y/N\n"):
            print_log("    Ok, flashing anyway!")
        else:
            raise WrongTargetSelected(message)


def reset_to_bootloader(port, detected_baud, reboot_cmd, target=""):
    import time
    import serial
    import SerialHelper

    print_header("======== RESET TO BOOTLOADER ========\n")
    s = serial.Serial(port=port, baudrate=detected_baud,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)
    rl = SerialHelper.SerialHelper(s, 3., delimiters=["\n"])
    rl.clear()
    rl.write(reboot_cmd)
    time.sleep(.3)
    # Wait target information if available
    rx_target = rl.read_line().strip()
    s.close()
    validate_reported_rx_type(rx_target, target)


if __name__ == '__main__':
    print("CRC: %s" % get_init_seq('CRSF', [ord(x) for x in 'R9MM']))
    print("CRC: %s" % get_init_seq('CRSF', 'R9MM'))
