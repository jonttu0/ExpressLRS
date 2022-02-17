import sys
import argparse
import serials_find
import bootloader
import upload_passthrough_xmodem as uploader_xmodem
import upload_passthrough_esptool as uploader_esp
from console_log import *


def on_upload(source, target, env):
    target_name = env['PIOENV'].upper()
    platform = env.get('PIOPLATFORM', '')
    envkey = None
    ghst = False
    firmware_path = str(source[0])

    elrs_baudrate = bootloader.BAUDRATE_ELRS_PROTO
    flags = env.get('BUILD_FLAGS', [])
    for flag in flags:
        if 'PROTOCOL_ELRS_RX_BAUDRATE=1' in flag:
            elrs_baudrate = eval(flag.split("=")[1])
    for flag in flags:
        if 'PROTOCOL_ELRS_TO_FC=' in flag and 0 < eval(flag.split("=")[1]):
            print_log("ELRS build option detected. Change upload speed to %u" % elrs_baudrate)
            env['UPLOAD_SPEED'] = elrs_baudrate

    upload_port = env.get('UPLOAD_PORT', None)
    if upload_port is None:
        upload_port = serials_find.get_serial_port()
    upload_speed = env.get('UPLOAD_SPEED', None)
    #if "_RX_" in target_name:
    #    upload_speed = env.GetProjectOption("upload_speed_passthrough", upload_speed)
    if upload_speed is None:
        upload_speed = bootloader.BAUDRATE_CRSF

    upload_flags = env.get('UPLOAD_FLAGS', [])
    for line in upload_flags:
        flags = line.split()
        for flag in flags:
            if "GHST=" in flag:
                ghst = eval(flag.split("=")[1])
            elif "BL_KEY=" in flag:
                envkey = flag.split("=")[1]

    try:
        if platform in ['ststm32']:
            uploader_xmodem.uart_upload(upload_port, firmware_path, upload_speed,
                                        ghst, key=envkey, target=target_name)
        elif platform in ['espressif8266', 'espressif32']:
            uploader_esp.uart_upload(upload_port, firmware_path, upload_speed,
                                     key=envkey, target=target_name)
    except Exception as e:
        print_error("{0}\n".format(e))
        return -1
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Upload firmware using FC passthrough")
    parser.add_argument('firmware', metavar='firmware.bin', type=str, nargs=1,
        help='The target firmware that is going to be uploaded')
    parser.add_argument("-b", "--baud", type=int,
        default=bootloader.BAUDRATE_CRSF,
        help="Baud rate for passthrough communication")
    parser.add_argument("-p", "--port", type=str, default="auto",
        help="Override serial port autodetection and use PORT")
    parser.add_argument("-t", "--target", type=str,
        help="The target firmware that is going to be uploaded")
    parser.add_argument("-k", "--key", type=str,
        help="The bootloader target key.")
    parser.add_argument("-hd", "--half-duplex", action="store_true",
        dest="half_duplex", help="Use half duplex mode")
    parser.add_argument("--esp", action="store_false", dest="is_stm",
        help="Defines flash target type is ESP. Default is STM32")
    args = parser.parse_args()

    if args.port == "auto":
        args.port = serials_find.get_serial_port()

    if args.is_stm:
        uploader_xmodem.uart_upload(args.port, args.firmware[0], args.baud,
                                    args.half_duplex, key=args.key,
                                    target=args.target)
    else:
        uploader_esp.uart_upload(args.port, args.firmware[0], args.baud,
                                 key=args.key, target=args.target)
