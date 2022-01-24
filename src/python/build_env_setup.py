Import("env", "projenv")
import stlink
import UARTupload
import opentx
import upload_via_esp8266_backpack
import esp_compress
import upload_passthrough_edgetx


FAIL = '\033[91m'
#FAIL = '\033[47;31m'

tgt_WIFI = "wifi"
tgt_STLINK = "stlink"
tgt_PASSTHROUGH = "passthrough"
tgt_DFU = "dfu"

platform = env.get('PIOPLATFORM', '')
target_name = env['PIOENV'].upper()


def find_build_flag(search):
    if not search:
        return
    for flag in env['BUILD_FLAGS']:
        if search in flag:
            return str(flag)
    return ""


# don't overwrite if custom command defined
#if stm and "$UPLOADER $UPLOADERFLAGS" in env.get('UPLOADCMD', '$UPLOADER $UPLOADERFLAGS'):
if platform in ['ststm32']:
    features = env.GetProjectOption("features", "")

    board = env.BoardConfig()
    hwids_list = board.get("build.hwids", [])
    upload_protocols = board.get("upload.protocols", "")

    # Default to ST-Link uploading
    # Note: this target is also used to build 'firmware.elrs' binary
    #       for handset flashing
    env.Replace(UPLOADCMD=stlink.on_upload)
    env.AddCustomTarget(tgt_STLINK,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        [stlink.on_upload],
        title="Upload via ST-Link", description="")
    if "_TX" in target_name or "_HANDSET" in target_name:
        upload_flags = env.GetProjectOption("upload_flags", "")
        wifi_targets = []
        if "_TX" in target_name and "BOOTLOADER=" in " ".join(upload_flags):
            # Generate 'firmware.elrs' file to be used for uploading (prio to bin file).
            # This enables support for OTA updates with the "overlay double" R9M module
            # bootloader. Bin file upload also requires the correct FLASH_OFFSET value
            # and that can be ignored when '.elrs' file is used.
            env.AddPostAction("buildprog", opentx.gen_elrs)
            env.AddPreAction("upload", opentx.gen_elrs)
            wifi_targets.append(opentx.gen_elrs)
        if find_build_flag("HAS_WIFI_BACKPACK") or \
                "HAS_WIFI_BACKPACK" in features or \
                "wifi" in upload_protocols:
            wifi_targets.append(upload_via_esp8266_backpack.on_upload)
            # the target can use WIFI (backpack logger) upload
            env.AddCustomTarget(tgt_WIFI,
                ["$BUILD_DIR/${PROGNAME}.bin"],
                wifi_targets,
                title="Upload via WiFi", description="")
    elif "_RX" in target_name:
        # Check whether the target is using FC passthrough upload (receivers)
        env.AddCustomTarget(tgt_PASSTHROUGH,
            ["$BUILD_DIR/${PROGNAME}.bin"],
            [UARTupload.on_upload],
            title="Upload via FC Passthrough", description="")

    if "dfu" in upload_protocols:
        if not hwids_list:
            raise SystemExit("DFU enabled but no HW IDs available!")
        hwids = [s.replace("0x", "") for s in hwids_list[0]]
        command = 'dfu-util -d %s -s %s:leave -D "$BUILD_DIR/${PROGNAME}.bin"' % (
            ":".join(hwids), board.get("upload.offset_address", "0x08001000"))
        env.AddCustomTarget(tgt_DFU,
            dependencies=["$BUILD_DIR/${PROGNAME}.bin"],
            actions=command,
            title="Upload via DFU", description="")

elif platform in ['espressif8266']:
    env.AddPostAction("buildprog", esp_compress.compressFirmware)
    env.AddPreAction("${BUILD_DIR}/spiffs.bin",
                     [esp_compress.compress_files])
    env.AddPreAction("${BUILD_DIR}/${ESP8266_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_files])
    env.AddPostAction("${BUILD_DIR}/${ESP8266_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_fs_bin])
    env.AddCustomTarget(tgt_WIFI,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        [esp_compress.compressFirmware, upload_via_esp8266_backpack.on_upload],
        title="Upload via WiFi", description="")
    if "_RX" in target_name:
        # Check whether the target is using FC passthrough upload (receivers)
        env.AddCustomTarget(tgt_PASSTHROUGH,
            ["$BUILD_DIR/${PROGNAME}.bin"],
            [esp_compress.compressFirmware, UARTupload.on_upload],
            title="Upload via FC Passthrough", description="")

elif platform in ['espressif32']:
    env.AddCustomTarget(tgt_WIFI,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        [esp_compress.compressFirmware, upload_via_esp8266_backpack.on_upload],
        title="Upload via WiFi", description="")
    if "_ETX" in target_name:
        env.AddPreAction("upload", upload_passthrough_edgetx.init_passthrough)

else:
    raise SystemExit(FAIL + "\nNot supported platfrom! '%s'\n" % platform)


if False:
    if "_TX_" in target_name:
        env.SetDefault(UPLOAD_PORT="elrs_tx.local")
    else:
        env.SetDefault(UPLOAD_PORT="elrs_rx.local")

