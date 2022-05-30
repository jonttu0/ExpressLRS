import serial
import time
import streamexpect
from console_log import *


def EdgeTxPassthroughEnable(port, requestedBaudrate):
    print_header("======== PASSTHROUGH INIT ========")
    print_log("  Trying to initialize %s @ %s" % (port, requestedBaudrate))

    s = serial.Serial(port=port, baudrate=requestedBaudrate,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)

    try:
        with streamexpect.wrap(s) as rl:
            rl.flush()
            rl.write(b"set pulses 0\n")
            rl.expect_bytes(b"set: ", timeout=1.0)
            rl.expect_bytes(b"> ", timeout=1.0)
            rl.write(b"set rfmod 0 power off\n")
            rl.expect_bytes(b"set: ", timeout=1.0)
            rl.expect_bytes(b"> ", timeout=1.0)
            time.sleep(.5)
            rl.write(b"set rfmod 0 bootpin 1\n")
            rl.expect_bytes(b"set: ", timeout=1.0)
            rl.expect_bytes(b"> ", timeout=1.0)
            time.sleep(.1)
            rl.write(b"set rfmod 0 power on\n")
            rl.expect_bytes(b"set: ", timeout=1.0)
            rl.expect_bytes(b"> ", timeout=1.0)
            time.sleep(.1)
            rl.write(b"set rfmod 0 bootpin 0\n")
            rl.expect_bytes(b"set: ", timeout=1.0)
            rl.expect_bytes(b"> ", timeout=1.0)

            cmd = "serialpassthrough rfmod 0 %s" % requestedBaudrate

            print_log("  Enabling serial passthrough...")
            print_log("    CMD: '%s'" % cmd)
            rl.write(cmd.encode("utf-8"))
            rl.write(b'\n')
            time.sleep(.2)
    except streamexpect.ExpectTimeout:
        raise SystemExit("[ERROR] PASSTHROUGH init failed!")

    s.close()
    print_info("======== PASSTHROUGH DONE ========")


def init_passthrough(source, target, env):
    env.AutodetectUploadPort([env])
    EdgeTxPassthroughEnable(env['UPLOAD_PORT'], env['UPLOAD_SPEED'])
