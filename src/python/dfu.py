import os
from platformio.util import get_systype
from SCons.Script import ARGUMENTS
import stlink


def get_commands(env, firmware):
    pioplatform = env.PioPlatform()
    board_cfg = env.BoardConfig()

    offset_address = int(board_cfg.get("upload.offset_address", "0x%08X" % stlink.FLASH_BASE), 16)

    flags = stlink.parse_flags(env)
    flash_start = flags.get("addr booloader", stlink.FLASH_BASE)
    app_start = flags.get("addr app", offset_address)
    bootloader = flags.get("bootloader bin", None)

    dfu_util_path = os.path.join(
        pioplatform.get_package_dir("tool-dfuutil") or "",
        "bin", "dfu-util")

    options = ""
    if pioplatform.name =='arterytekat32':
        default = [["2e3c", "df11"]]
        options = "-a 0"
    else:
        default = []
    hwids_list = board_cfg.get("build.hwids", default)
    if not hwids_list:
        raise SystemExit("DFU enabled but no HW IDs available!")
    hwids = ",".join(["%s:%s" % (hwid[0].replace("0x",""), hwid[1].replace("0x", "")) for hwid in hwids_list])

    dfu_cmd_fmt = f'{dfu_util_path} {options} -d {hwids} -s %s -D "%s"'

    BL_CMD = ""
    if bootloader is not None:
        # BL_CMD = dfu_cmd_fmt % (f"0x{flash_start:08X}:mass-erase:force", bootloader)
        BL_CMD = dfu_cmd_fmt % (f"0x{flash_start:08X}", bootloader)
        print("BL_CMD: {}".format(BL_CMD))
    APP_CMD = dfu_cmd_fmt % (f"0x{app_start:08X}:leave", firmware)
    print("APP_CMD: {}".format(APP_CMD))
    return BL_CMD, APP_CMD


def exect_commands(env, BL_CMD, APP_CMD):
    retval = 0
    # flash bootloader
    if BL_CMD:
        retval = env.Execute(BL_CMD)
    # flash application
    if retval == 0 and APP_CMD:
        retval = env.Execute(APP_CMD)
    return retval


def on_upload(source, target, env):
    firmware_path = str(source[0])
    BL_CMD, APP_CMD = get_commands(env, firmware_path)
    return exect_commands(env, BL_CMD, APP_CMD)
