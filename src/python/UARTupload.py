import sys
import serials_find
import bootloader
import upload_passthrough_xmodem as uploader_xmodem
import upload_passthrough_esptool as uploader_esp


def dbg_print(line=''):
    sys.stdout.write(line)
    sys.stdout.flush()
    return


def on_upload(source, target, env):
    target_name = env['PIOENV'].upper()
    platform = env.get('PIOPLATFORM', '')
    envkey = None
    ghst = False
    firmware_path = str(source[0])

    flags = env.get('BUILD_FLAGS', [])
    for flag in flags:
        if 'PROTOCOL_ELRS_TO_FC=1' in flag:
            print("ELRS protocol detected. Change upload speed to %u" % bootloader.BAUDRATE_ELRS_PROTO)
            env['UPLOAD_SPEED'] = bootloader.BAUDRATE_ELRS_PROTO

    upload_port = env.get('UPLOAD_PORT', None)
    if upload_port is None:
        upload_port = serials_find.get_serial_port()
    upload_speed = env.get('UPLOAD_SPEED', None)
    if "_RX_" in target_name:
        upload_speed = env.GetProjectOption("upload_speed_passthrough", upload_speed)
    if upload_speed is None:
        upload_speed = bootloader.BAUDRATE_DEFAULT

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
        dbg_print("{0}\n".format(e))
        return -1
    return 0


if __name__ == '__main__':
    filename = 'firmware.bin'
    baudrate = bootloader.BAUDRATE_DEFAULT
    is_stm = True
    try:
        filename = sys.argv[1]
    except IndexError:
        dbg_print("Filename not provided, going to use default firmware.bin")

    if 2 < len(sys.argv):
        port = sys.argv[2]
    else:
        port = serials_find.get_serial_port()

    if 3 < len(sys.argv):
        baudrate = sys.argv[3]

    if 4 < len(sys.argv):
        is_stm = eval(sys.argv[4])
    if is_stm:
        uploader_xmodem.uart_upload(port, filename, baudrate)
    else:
        uploader_esp.uart_upload(port, filename, baudrate)
