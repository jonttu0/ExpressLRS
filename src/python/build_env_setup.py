Import("env", "projenv")
import dfu
import stlink
import UARTupload
import opentx
import upload_via_wifi
import esp_compress
import upload_passthrough_edgetx
import shutil, os


FAIL = '\033[91m'

tgt_UART = "uart"
tgt_WIFI = "wifi"
tgt_STLINK = "stlink"
tgt_OPENOCD = "openocd"
tgt_PASSTHROUGH = "passthrough"
tgt_DFU = "dfu"

# pioplatform = env.PioPlatform()
# platform = pioplatform.name
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
if platform in ['ststm32', 'arterytekat32']:
    features = env.GetProjectOption("custom_features", "")
    # default_protocol = env.GetProjectOption("upload_protocol", "")

    board_config = env.BoardConfig()
    upload_protocols = board_config.get("upload.protocols", "")

    # Default to ST-Link uploading
    default_upload_cmd = stlink.on_upload

    env.AddCustomTarget(tgt_STLINK,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        [stlink.on_upload],
        title="Upload via ST-Link", description="")
    env.AddCustomTarget(tgt_OPENOCD,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        [stlink.on_upload_openocd],
        title="Upload via OpenOCD", description="")

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
            # wifi_targets.append(opentx.gen_elrs)
        if find_build_flag("HAS_WIFI_BACKPACK") or \
                "HAS_WIFI_BACKPACK" in features or \
                "wifi" in upload_protocols:
            wifi_targets.append(upload_via_wifi.on_upload)
            # the target can use WIFI (backpack logger) upload
            env.AddCustomTarget(tgt_WIFI,
                ["$BUILD_DIR/${PROGNAME}.bin"],
                wifi_targets,
                title="Upload via WiFi", description="")
            # Default to ST-Link uploading
            default_upload_cmd = wifi_targets

    elif "_RX" in target_name:
        # Default to passthrough uploading
        default_upload_cmd = UARTupload.on_upload

        # Check whether the target is using FC passthrough upload (receivers)
        env.AddCustomTarget(tgt_PASSTHROUGH,
            ["$BUILD_DIR/${PROGNAME}.bin"],
            [UARTupload.on_upload],
            title="Upload via FC Passthrough", description="")

    if "dfu" in upload_protocols or platform =='arterytekat32':
        env.AddCustomTarget(tgt_DFU,
            ["$BUILD_DIR/${PROGNAME}.bin"],
            [dfu.on_upload],
            title="Upload via DFU", description="")

    # Override default upload command
    if default_upload_cmd is not None:
        env.Replace(UPLOADCMD=default_upload_cmd)

elif platform in ['espressif8266']:
    env.AddPostAction("buildprog", esp_compress.compressFirmware)
    env.AddPreAction("${BUILD_DIR}/spiffs.bin",
                     [esp_compress.compress_files])
    env.AddPreAction("${BUILD_DIR}/${ESP8266_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_files])
    env.AddPostAction("${BUILD_DIR}/${ESP8266_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_fs_bin])

    # Move current command to custom targets
    env.AddCustomTarget(tgt_UART,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        env.get("UPLOADCMD", ""),
        title="Upload via UART", description="")
    # Add WiFi upload custom target
    wifi_targets = [esp_compress.compressFirmware, upload_via_wifi.on_upload]
    env.AddCustomTarget(tgt_WIFI,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        wifi_targets,
        title="Upload via WiFi", description="")
    # Default is wifi upload
    env.Replace(UPLOADCMD=wifi_targets)
    if "_RX" in target_name:
        # Check whether the target is using FC passthrough upload (receivers)
        env.AddCustomTarget(tgt_PASSTHROUGH,
            ["$BUILD_DIR/${PROGNAME}.bin"],
            [esp_compress.compressFirmware, UARTupload.on_upload],
            title="Upload via FC Passthrough", description="")

elif platform in ['espressif32']:
    env.AddPostAction("buildprog", esp_compress.compressFirmware)
    env.AddPreAction("${BUILD_DIR}/spiffs.bin",
                     [esp_compress.compress_files])
    env.AddPreAction("${BUILD_DIR}/${ESP32_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_files])
    env.AddPostAction("${BUILD_DIR}/${ESP32_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_fs_bin])

    env.AddCustomTarget(tgt_WIFI,
        ["$BUILD_DIR/${PROGNAME}.bin"],
        [esp_compress.compressFirmware, upload_via_wifi.on_upload],
        title="Upload via WiFi", description="")
    if "_ETX" in target_name:
        env.AddPreAction("upload", upload_passthrough_edgetx.init_passthrough)
    if "LOGGER_" in target_name:
        def copy_bootfile(source, target, env):
            BUILD_DIR = env.subst("$BUILD_DIR")
            FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
            shutil.copyfile(FRAMEWORK_DIR + "/tools/partitions/boot_app0.bin", BUILD_DIR + "/boot_app0.bin")
            image_name = env.subst("$PROGNAME")
            shutil.copyfile(os.path.join(BUILD_DIR, image_name + ".bin"), os.path.join(BUILD_DIR, "firmware.bin"))

        env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_bootfile)

else:
    raise SystemExit(FAIL + "\nNot supported platfrom! '%s'\n" % platform)
