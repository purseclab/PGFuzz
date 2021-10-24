import time
from subprocess import *
import os

PGFUZZ_HOME = os.getenv("PGFUZZ_HOME")

if PGFUZZ_HOME is None:
    raise Exception("PGFUZZ_HOME environment variable is not set!")

ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

if ARDUPILOT_HOME is None:
    raise Exception("ARDUPILOT_HOME environment variable is not set!")

open("restart.txt", "w").close()

c = 'gnome-terminal -- python2 ' + PGFUZZ_HOME + 'ArduPilot/open_simulator.py &'
handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

time.sleep(45)
c = 'gnome-terminal -- python2 ' + PGFUZZ_HOME + 'ArduPilot/fuzzing.py &'
handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

while True:
	time.sleep(1)

	f = open("restart.txt", "r")

	if f.read() == "restart":
		f.close()
		open("restart.txt", "w").close()

		c = 'gnome-terminal -- python2 ' + PGFUZZ_HOME + 'ArduPilot/open_simulator.py &'
		handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	
