import subprocess, os


def on_upload(source, target, env):
    platform = env.get('PIOPLATFORM', '')
    source_file = str(source[0])
    isstm = platform in ['ststm32']
    target_name = env['PIOENV'].upper()

    if not isstm and platform in ['espressif32']:
        # Update ESP transmitter
        upload_addr = ['elrs_tx.local', 'elrs_tx']
    elif not isstm and "LOGGER_" not in target_name and platform in ['espressif8266']:
        # Update ESP receiver
        upload_addr = ['elrs_rx.local', 'elrs_rx']
    else:
        # Update STM receiver or backpack logger
        upload_addr = ['elrs_logger.local', 'elrs_logger']

    if "backpack.bin" in source_file:
        # Logger firmware update
        elrs_bin_target = source_file + ".gz"
        if not os.path.exists(source_file + ".gz"):
            # compressed binary does not exist
            elrs_bin_target = source_file
    else:
        bin_path = os.path.dirname(source_file)
        elrs_bin_target = os.path.join(bin_path, 'firmware.elrs')
        if not os.path.exists(elrs_bin_target):
            elrs_bin_target = os.path.join(bin_path, 'firmware.bin')
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

    # Upload address can be given as a --upload-port
    upload_port = env.get('UPLOAD_PORT', None)
    if upload_port is not None:
        upload_addr = [upload_port]

    for addr in upload_addr:
        addr = "http://%s/%s" % (addr, ['update', 'upload'][isstm])
        print(" ** UPLOADING TO: %s" % addr)
        try:
            subprocess.check_call(cmd + [addr])
            print()
            print("** UPLOAD SUCCESS. Flashing in progress.")
            print("** Please wait for LED to resume blinking before disconnecting power")
            return
        except subprocess.CalledProcessError:
            print("FAILED!")

    raise SystemExit("WIFI upload FAILED!")
