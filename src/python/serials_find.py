import serial
import sys, glob
from console_log import *


def serial_ports():
    """ Lists serial port names

        :raises Exception:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    result = []
    ports = []

    try:
        from serial.tools.list_ports import comports
        if comports:
            print_log("  ** Searching flight controllers **")
            __ports = list(comports())
            for port in __ports:
                if (port.manufacturer and port.manufacturer in ['FTDI', 'Betaflight', "Artery"]) or \
                        (port.product and ("STM32" in port.product or "AT32" in port.product)) or \
                        (port.vid and port.vid in [0x0483, 0x2e3c]):
                    print_log("      > FC found from '%s'" % port.device)
                    ports.append(port.device)
    except ImportError:
        pass

    if not ports:
        print_warning("  ** No FC found, find all ports **")

        platform = sys.platform.lower()
        if platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif platform.startswith('linux') or platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            #ports = glob.glob('/dev/tty[A-Za-z]*')
            # List all ttyACM* and ttyUSB* ports only
            ports = glob.glob('/dev/ttyACM*')
            ports.extend(glob.glob('/dev/ttyUSB*'))
        elif platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.usbmodem*')
            ports.extend(glob.glob('/dev/tty.SLAB*'))
        else:
            raise SystemExit('Unsupported platform')

    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException) as error:
            if "permission denied" in str(error).lower():
                raise SystemExit("You don't have persmission to use serial port!")
            pass
    result.reverse()
    return result

def get_serial_port(debug=True):
    result = serial_ports()
    if not result:
        raise SystemExit('No valid serial port detected or port already open')
    if debug:
        print_log("  [DEBUG] Detected the following serial ports on this system:")
        for port in result:
            print_log("    %s" % port)
        print_log("")
    return result[0]

if __name__ == '__main__':
    results = get_serial_port(True)
    print_log("Found: %s" % (results, ))
