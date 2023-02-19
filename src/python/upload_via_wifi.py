import subprocess, os
from console_log import *


def on_upload(source, target, env):
    platform = env.get('PIOPLATFORM', '')
    source_file = str(source[0])
    isstm = platform in ['ststm32']
    target_name = env['PIOENV'].upper()
    upload_addr = env.GetProjectOption("custom_dns", "").split()
    if not upload_addr:
        if not isstm and platform in ['espressif32']:
            # Update ESP transmitter
            upload_addr = ['elrs_tx.local', 'elrs_tx']
        elif not isstm and "LOGGER_" not in target_name and platform in ['espressif8266']:
            # Update ESP receiver
            upload_addr = ['elrs_rx.local', 'elrs_rx']
        else:
            # Update STM receiver or backpack logger
            upload_addr = ['elrs_logger.local', 'elrs_logger']
    elrs_bin_name = ""
    if "backpack.bin" in source_file:
        # Logger firmware update
        elrs_bin_target = source_file + ".gz"
        if not os.path.exists(source_file + ".gz"):
            # compressed binary does not exist
            elrs_bin_target = source_file
    else:
        bin_path = os.path.dirname(source_file)
        elrs_bin_name = "firmware.elrs"
        elrs_bin_target = os.path.join(bin_path, elrs_bin_name)
        if not os.path.exists(elrs_bin_target):
            elrs_bin_name = "firmware.bin"
            elrs_bin_target = os.path.join(bin_path, elrs_bin_name)
    # Check if the binary exits
    if not os.path.exists(elrs_bin_target):
        raise SystemExit("No valid binary found!")

    cmd = ["curl", "--max-time", "60",
           "--retry", "2", "--retry-delay", "1",
           "-F", "data=@%s" % (elrs_bin_target,)]
    # resolve bootloader offset for STM32
    if isstm:
        app_start = 0
        # Parse upload flags:
        upload_flags = env.get('UPLOAD_FLAGS', [])
        for line in upload_flags:
            flags = line.split()
            for flag in flags:
                if "VECT_OFFSET=" in flag:
                    offset = flag.split("=")[1]
                    if "0x" in offset:
                        offset = int(offset, 16)
                    else:
                        offset = int(offset, 10)
                    app_start = offset
        cmd += ["-F", "flash_address=0x%X" % (app_start,)]
        if elrs_bin_name:
            cmd += ["-F", "firmware=%s" % (elrs_bin_name,)]

    # Upload address can be given as a --upload-port
    upload_port = env.get('UPLOAD_PORT', None)
    if upload_port is not None:
        upload_addr = [upload_port]
    print("cmd: %s" % cmd)
    for addr in upload_addr:
        addr = "http://%s/%s" % (addr, ['doUpdate', 'upload'][isstm])
        print_header("  == UPLOADING TO: %s ==" % addr)
        try:
            subprocess.check_call(cmd + [addr])
            print_info("  UPLOAD SUCCESS. Flashing in progress.")
            print_warning("    !! Please wait for LED to resume blinking before disconnecting power")
            return
        except subprocess.CalledProcessError:
            print_error("  !! UPLOAD FAILED !!")

    raise SystemExit("WIFI upload FAILED!")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description="Upload firmware using FC passthrough")
    parser.add_argument('firmware', metavar='firmware.bin', type=str, nargs=1,
        help='The target firmware that is going to be uploaded')
    parser.add_argument("-p", "--port", type=str, default="",
        help="Override serial port autodetection and use PORT")
    parser.add_argument("-t", "--target", type=str, default="",
        help="The target firmware that is going to be uploaded")
    parser.add_argument("--platform", type=str, default="stm32",
        help="Defines flash target type. Default: stm32.")
    parser.add_argument("--offset", type=str, default="0x4000",
        help="Defines flash offset for STM32")
    args = parser.parse_args()

    if not args.target:
        raise SystemExit("Target is mandatory")
    if not args.port:
        raise SystemExit("Port is mandatory")

    platforms = {
        "stm32": "ststm32",
        "esp82": "espressif8266",
        "esp32": "espressif32",
    }

    _env = {
        "UPLOAD_PORT": args.port,
        "PIOPLATFORM": platforms[args.platform],
        "PIOENV": args.target,
        "UPLOAD_FLAGS": [
            "VECT_OFFSET=%s" % args.offset,
        ]
    }

    on_upload(args.firmware, None, _env)
