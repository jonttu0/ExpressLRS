Import("env")
import os, re
#import fhss_random
import hashlib
try:
    import git
except ImportError:
    env.Execute('"$PYTHONEXE" -m pip install GitPython')
    try:
        import git
    except ImportError:
        git = None
from console_log import *


# print(env.Dump())
target_name = env.get('PIOENV', '')
my_uid_final = [0] * 6


def find_build_flag(search):
    if not search:
        return
    for flag in env['BUILD_FLAGS']:
        if search in flag:
            return flag
    return ""

def validate_domains():
    build_flags = env['BUILD_FLAGS']
    ISM2400 = DUAL_MODE = False
    for flag in build_flags:
        if "DOMAIN_24GHZ" in flag:
            ISM2400 = True
        elif "DOMAIN_BOTH" in flag:
            DUAL_MODE = True
    domains_found = []
    domains_found_ism = []
    for flag in build_flags:
        if "Regulatory_Domain" in flag:
            if "_ISM_2400" in flag:
                if not DUAL_MODE and not ISM2400:
                    continue
                domains_found_ism.append(flag)
            else:
                if not DUAL_MODE and ISM2400:
                    continue
                domains_found.append(flag)
    if not len(domains_found_ism) and not len(domains_found):
        raise Exception("[ERROR] 'Regulatory_Domain' is not defined")
    if len(domains_found_ism) > 2:
        if "-DDRegulatory_Domain_ISM_2400_800kHz" in domains_found_ism and \
            "-DRegulatory_Domain_ISM_2400" in domains_found_ism:
            raise Exception("[ERROR] Only one 'Regulatory_Domain' is allowed")
    if len(domains_found) > 1:
        raise Exception("[ERROR] Only one 'Regulatory_Domain' is allowed")
    if domains_found:
        build_flags.append("-DRADIO_SX127x=1")
    if domains_found_ism:
        build_flags.append("-DRADIO_SX128x=1")

        for domain in domains_found_ism:
            if "_FLRC" in domain:
                build_flags.append("-DRADIO_SX128x_FLRC=1")
            elif "_800kHz" in domain:
                build_flags.append("-DRADIO_SX128x_BW800=1")


def parse_env_defines():
    build_flags = env['BUILD_FLAGS']
    elrs_flags = env.get('ENV', {}).get('EXPRESSLRS_FLAGS', "")
    if elrs_flags:
        print_info(f" == ENV FLAGS DEFINED ==")
        # check if 'Regulatory_Domain_' is set
        if 'Regulatory_Domain_' in elrs_flags:
            # ... and remove existing values
            build_flags_copy = list(build_flags)
            for flag in build_flags_copy:
                if 'Regulatory_Domain_' in flag:
                    print_warning(f"  Regulatory_Domain '{flag}' removed")
                    build_flags.remove(flag)
        # add flags
        elrs_flags = elrs_flags.split()
        for flag in elrs_flags:
            if flag in build_flags:
                print_warning(f"  flag '{flag}' removed")
                build_flags.remove(flag)
            print_info(f"  flag '{flag}' added")
            build_flags.append(flag)

def parse_flags(path):
    global my_uid_final
    build_flags = env['BUILD_FLAGS']
    try:
        with open(path, "r") as _f:
            for line in _f:
                define = line.strip()
                if define.startswith("-D"):
                    is_uid = "MY_PHRASE" in define or "MY_BINDING_PHRASE" in define or "MY_UID" in define

                    define_key = define.split("=")[0]
                    build_flags_copy = list(build_flags)
                    for flag in build_flags_copy:
                        if define_key in flag or (is_uid and ("MY_PHRASE" in flag or "MY_UID" in flag)):
                            ###print("remove %s (%s, %s)" % (flag, define_key, define))
                            # remove value and it will be replaced
                            build_flags.remove(flag)
                            break

                    if "MY_BINDING_PHRASE" in define:
                        bindingPhraseHash = hashlib.md5(define.encode()).digest()
                        UIDbytes = ",".join(list(map(str, bindingPhraseHash))[0:6])
                        my_uid_final = [eval(x) for x in UIDbytes.split(",")]
                        define = "-DMY_UID=" + UIDbytes
                    elif "MY_PHRASE" in define:
                        define = define.split("=")[1]
                        define = define.replace('"', '').replace("'", "")
                        key = define.replace("-D", "")
                        if len(define) < 8:
                            raise Exception("MY_PHRASE must be at least 8 characters long")
                        md5 = hashlib.md5(key.encode()).hexdigest()
                        # print("Hash value: %s" % md5)
                        #my_uid_final = my_uid = ["0x%02X"%ord(r) for r in md5[:6]]
                        my_uid_final = my_uid = [int(md5[i:(i+2)],16) for i in range(0, 12, 2)]
                        define = "-DMY_UID=" + ",".join(["0x%02X"%x for x in my_uid])
                        # print("Calculated UID[6] = {%s}" % ",".join(my_uid))
                    elif "MY_UID" in define:
                        _define = define.replace("-DMY_UID=", "").split(",")
                        if len(_define) != 6:
                            raise Exception("UID must be 6 bytes long")
                        my_uid_final = [eval(x) for x in _define]
                    build_flags.append(define)
    except IOError:
        return False
    return True


if not parse_flags("user_defines.txt"):
    err = "\n\033[91m[ERROR] File 'user_defines.txt' does not exist\n"
    raise Exception(err)
# try to parse user private params
parse_flags("user_defines_private.txt")
parse_env_defines()
validate_domains()

# print UID
print_info("------------------------")
print_log("[INFO]", nl=False)
print_log(" My UID:")
print_log("  UID[6] = {%s}" % ",".join(["0x%02X"%x for x in my_uid_final]))
print_log("  set expresslrs_uid = %s" % ",".join([str(x) for x in my_uid_final]))
print_info("------------------------")

sha_string = "unknown"
sha = None
if git:
    try:
        git_repo = git.Repo(
            os.path.abspath(os.path.join(os.getcwd(), os.pardir)),
            search_parent_directories=False)
        git_root = git_repo.git.rev_parse("--show-toplevel")
        ExLRS_Repo = git.Repo(git_root)
        # git describe --match=NeVeRmAtCh --always --abbrev=6 --dirty
        sha_string = ExLRS_Repo.git.describe('--dirty', '--abbrev=6', '--always', '--match=NeVeRmAtCh')
        if 'dirty' in sha_string:
            env['BUILD_FLAGS'].append("-DLATEST_COMMIT_DIRTY=1")
        sha = ",".join(["0x%s" % x for x in sha_string[:6]])
    except git.InvalidGitRepositoryError:
        pass
if sha is None:
    if os.path.exists("VERSION"):
        with open("VERSION") as _f:
            data = _f.readline()
            _f.close()
        sha_string = data.split()[1].strip()
        sha = ",".join(["0x%s" % x for x in sha_string[:6]])
    else:
        sha = "0,0,0,0,0,0"
env.Append(LATEST_COMMIT = sha_string)
env.Append(TARGET_NAME = target_name)
print_log("[INFO] Current version: '%s'" % sha_string)
# print_log("Current SHA: %s" % sha)
env['BUILD_FLAGS'].append("-DLATEST_COMMIT="+sha)
env['BUILD_FLAGS'].append('-DLATEST_COMMIT_STR="\\"%s\\""' % sha_string)
env['BUILD_FLAGS'].append(f"-DTARGET_NAME={target_name}")
print_info("------------------------")

header = f"src/include/target_{target_name}.h"
if os.path.exists(header) and \
        not find_build_flag("-include src/include/target_"):
    env['BUILD_FLAGS'].append(f"-include {header}")
    print_warning("[NOTE] target include header file added automatically!")
    print_info("------------------------")

#print_log("\n[INFO] build flags: %s\n" % env['BUILD_FLAGS'])
print_log("[INFO] build flags:")
for flag in env['BUILD_FLAGS']:
    print_log("    %s" % flag)
print_info("------------------------")

#fhss_random.check_env_and_parse(env['BUILD_FLAGS'])

# Set upload_protovol = 'custom' for STM32 MCUs
#  otherwise firmware.bin is not generated
pioplatform = env.get('PIOPLATFORM', '')
if pioplatform in ['ststm32']:
    env['UPLOAD_PROTOCOL'] = 'custom'

    def print_src(node):
        print("Source: %s" % node)
        return node
    # env.AddBuildMiddleware(print_src, "*.c*")

    # Filter out unnecessary STM32 HAL files
    def filter_stm_hal_files(node):
        name = str(node)
        include = ["_flash", "_rcc", "_cortex", "_gpio", "_pwr", "_i2c", "_tim.c", "_hal.c"]
        if any(x in name for x in include):
            return node
        # print(f"Ignored, HAL: {name}")
        return None
    env.AddBuildMiddleware(filter_stm_hal_files, "*stm32*_hal_*.c*")

    # Filter out unnecessary STM32 LL files
    def filter_stm_ll_files(node):
        name = str(node)
        include = []
        if any(x in name for x in include):
            return node
        # print(f"Ignored, LL: {name}")
        return None
    env.AddBuildMiddleware(filter_stm_ll_files, "*stm32*_ll_*.c*")

elif pioplatform in ['arterytekat32']:
    env['UPLOAD_PROTOCOL'] = 'custom'
