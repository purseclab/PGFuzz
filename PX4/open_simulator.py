from subprocess import *
# import subprocess,time,psutil

import time
import os
import signal
import psutil
import sys

PX4_HOME = os.getenv("PX4_HOME")

if PX4_HOME is None:
    raise Exception("PX4_HOME environment variable is not set!")

print("PX4_HOME:%s" %PX4_HOME)

c = 'cd ' + PX4_HOME + ' && make clean && make distclean && make px4_sitl_default jmavsim'
#print("[DEBUG] Command:%s" %c)

#handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
handle = Popen(c, shell=True)

# os.killpg(os.getpgid(handle.pid), signal.SIGTERM)

while True:

    f = open("shared_variables.txt", "r")

    if f.read() == "reboot":
        open("shared_variables.txt", "w").close()

        fi = open("restart.txt", "w")
        fi.write("restart")
        fi.close()

        os.killpg(os.getpgid(handle.pid), signal.SIGTERM)

    time.sleep(1)
