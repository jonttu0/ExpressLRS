import subprocess, os


def on_upload(source, target, env):
    isstm = env.get('PIOPLATFORM', '') in ['ststm32']
    target_name = env['PIOENV'].upper()

    if "_TX_" in target_name:
        upload_addr = ['elrs_tx.local', 'elrs_tx']
    elif "_RX_" in target_name:
        upload_addr = ['elrs_rx.local', 'elrs_rx']
    else:
        upload_addr = ['elrs_logger.local', 'elrs_logger']

    app_start = 0 # eka bootloader offset

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

    firmware_path = str(source[0])
    bin_path = os.path.dirname(firmware_path)
    elrs_bin_target = os.path.join(bin_path, 'firmware.elrs')
    if not os.path.exists(elrs_bin_target):
        elrs_bin_target = os.path.join(bin_path, 'firmware.bin')
        if not os.path.exists(elrs_bin_target):
            raise SystemExit("No valid binary found!")

    cmd = ["curl", "--max-time", "60",
           "--retry", "2", "--retry-delay", "1",
           "-F", "data=@%s" % (elrs_bin_target,)]
    if isstm:
        cmd += ["-F", "flash_address=0x%X" % (app_start,)]

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
