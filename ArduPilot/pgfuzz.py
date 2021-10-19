import time
from subprocess import *


open("restart.txt", "w").close()

c = 'gnome-terminal -- python ~/pgfuzz/ArduPilot/open_simulator.py &'
handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

time.sleep(45)
c = 'gnome-terminal -- python ~/pgfuzz/ArduPilot/fuzzing.py &'
handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

while True:
	time.sleep(1)

	f = open("restart.txt", "r")

	if f.read() == "restart":
		f.close()
		open("restart.txt", "w").close()

		c = 'gnome-terminal -- python ~/pgfuzz/ArduPilot/open_simulator.py &'
		handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	
