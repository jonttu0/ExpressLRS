from console_log import *

Import("env")


remove_flags = [
    # "-lgcc", "-lstdc++"
]
for scope in ("ASFLAGS", "CCFLAGS", "LINKFLAGS"):
    for option in remove_flags:
        while option in env[scope]:
            env[scope].remove(option)
