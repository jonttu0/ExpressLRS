import os
import platform
from platformio.util import get_systype
from SCons.Script import ARGUMENTS


FLASH_BASE = 0x08000000


def parse_flags(env):
    flags = {}
    build_flags = env.GetProjectOption("build_flags", [])
    for line in build_flags:
        for flag in line.split():
            if "FLASH_APP_OFFSET=" in flag:
                offset = flag.split("FLASH_APP_OFFSET=")[1].replace("K", "")
                flags["addr app"] = FLASH_BASE + int(offset) * 1024
    # upload_flags = env.get('UPLOAD_FLAGS', [])
    upload_flags = env.GetProjectOption("upload_flags", [])
    for line in upload_flags:
        for flag in line.split():
            if "BOOTLOADER=" in flag:
                flags["bootloader bin"] = flag.split("=")[1]
            elif "VECT_OFFSET=" in flag:
                offset = flag.split("=")[1]
                flags["addr app"] = FLASH_BASE + int(offset, [10, 16]["0x" in offset])
    return flags


def get_commands(env, firmware):
    platform_name = platform.system().lower()

    BL_CMD = []
    APP_CMD = []

    flags = parse_flags(env)
    flash_start = flags.get("addr booloader", FLASH_BASE)
    app_start = flags.get("addr app", FLASH_BASE)
    bootloader = flags.get("bootloader bin", None)

    pioplatform = env.PioPlatform()

    TOOL = os.path.join(
        pioplatform.get_package_dir("tool-stm32duino") or "",
        "stlink",
        ["st-flash", "ST-LINK_CLI.exe"]["windows" in platform_name])

    if "windows" in platform_name:
        if bootloader is not None:
            BL_CMD = [TOOL, "-c SWD SWCLK=8 -P", bootloader, hex(flash_start)]
        APP_CMD = [TOOL, "-c SWD SWCLK=8 -P", firmware, hex(app_start), "-RST"]
    elif "linux" in platform_name or "darwin" in platform_name:
        if bootloader is not None:
            BL_CMD = [TOOL, "write", bootloader, hex(flash_start)]
        APP_CMD = [TOOL, "--reset", "write", firmware, hex(app_start)]
    elif "os x" in platform_name:
        raise SystemExit("\nOS X not supported at the moment\n")
    else:
        raise SystemExit("\nOperating system: "+ platform_name +  " is not supported.\n")

    return " ".join(BL_CMD), " ".join(APP_CMD)


def get_commands_openocd(env, firmware_path):
    flags = parse_flags(env)
    flash_start = flags.get("addr booloader", FLASH_BASE)
    app_start = flags.get("addr app", FLASH_BASE)
    bootloader_bin = flags.get("bootloader bin", None)

    os_type = platform.system().lower()

    pioplatform = env.PioPlatform()
    openocd_tool = \
        ["tool-openocd", "tool-openocd-at32"][pioplatform.name == "arterytekat32"]
    openocd_tool_dir = pioplatform.get_package_dir(openocd_tool) or ""

    openocd_exe = os.path.join(
        openocd_tool_dir,
        "bin-"+ get_systype(),
        "openocd.exe" if os_type == "windows" else "openocd")

    stlink_args = env.BoardConfig().get("debug.tools", {}).get("stlink").get(
        "server").get("arguments", [])
    stlink_args = [f'"{f}"' if " " in f else f'{f}' for f in stlink_args]

    openocd_args = [
        openocd_exe,
        "-d%d" % (2 if int(ARGUMENTS.get("PIOVERBOSE", 0)) else 1)
    ]
    openocd_args.extend(stlink_args)
    openocd_args = [
        f.replace("$PACKAGE_DIR", openocd_tool_dir)
        for f in openocd_args
    ]

    program_fmt = '"program {%s} 0x%08X verify reset; shutdown;"'

    BL_CMD = ""
    if bootloader_bin is not None:
        command = openocd_args + ["-c", program_fmt % (bootloader_bin, flash_start)]
        BL_CMD = command = " ".join(command)
        # print("BL_CMD: {}".format(BL_CMD))
    command = openocd_args + ["-c", program_fmt % (firmware_path, app_start)]
    APP_CMD = command = " ".join(command)
    # print("BL_CMD: {}".format(APP_CMD))
    return BL_CMD, APP_CMD


def exect_commands(env, BL_CMD, APP_CMD):
    retval = 0
    # flash bootloader
    if BL_CMD:
        # print("Cmd: {}".format(BL_CMD))
        retval = env.Execute(BL_CMD)
    # flash application
    if retval == 0 and APP_CMD:
        # print("Cmd: {}".format(APP_CMD))
        retval = env.Execute(APP_CMD)
    return retval


def on_upload(source, target, env):
    firmware_path = str(source[0])

    try:
        BL_CMD, APP_CMD = get_commands(env, firmware_path)
    except KeyError:
        BL_CMD, APP_CMD = get_commands_openocd(env, firmware_path)
    return exect_commands(env, BL_CMD, APP_CMD)


def on_upload_openocd(source, target, env):
    firmware_path = str(source[0])
    BL_CMD, APP_CMD = get_commands_openocd(env, firmware_path)
    return exect_commands(env, BL_CMD, APP_CMD)
