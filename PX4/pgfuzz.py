import time
from subprocess import *
import sys, os, getopt

def main(argv):

	# (Start) Parse command line arguments (i.e., input and output file)
	try:
		opts, args = getopt.getopt(argv, "hi:o:", ["ifile="])

	except getopt.GetoptError:
		print("pgfuzz.py -i <true/false>")
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-h':
			print("pgfuzz.py -i <true: Bounded input mutation / false: Unbounded input mutation>")
			sys.exit()
		elif opt in ("-i", "--ifile"):
			input_type = arg

	open("input_mutation_type.txt", "w").close()

	f_input_type = open("input_mutation_type.txt", "w")
	if input_type == 'true':
		print("User chooses bounded input mutation")
		f_input_type.write('true')
	else:
		print("User chooses unbounded input mutation")
		f_input_type.write('false')

	f_input_type.close()
	# (End) Parse command line arguments (i.e., input and output file)

	PGFUZZ_HOME = os.getenv("PGFUZZ_HOME")

	if PGFUZZ_HOME is None:
		raise Exception("PGFUZZ_HOME environment variable is not set!")

	PX4_HOME = os.getenv("PX4_HOME")

	if PX4_HOME is None:
		raise Exception("PX4_HOME environment variable is not set!")

	open("restart.txt", "w").close()
	open("iteration.txt", "w").close()

	c = 'gnome-terminal -- python2 ' + PGFUZZ_HOME + 'PX4/open_simulator.py &'
	handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

	time.sleep(50)
	c = 'gnome-terminal -- python2 ' + PGFUZZ_HOME + 'PX4/fuzzing.py -i ' + input_type + '&'
	handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

	iteration = 1
	f2 = open("iteration.txt", "w")
	f2.write(str(iteration))
	f2.close()

	while True:
		time.sleep(1)

		f = open("restart.txt", "r")

		if f.read() == "restart":
			f.close()
			open("restart.txt", "w").close()

			iteration = iteration + 1
			open("iteration.txt", "w").close()
			f2 = open("iteration.txt", "w")
			f2.write(str(iteration))
			f2.close()

			c = 'gnome-terminal -- python2 ' + PGFUZZ_HOME + 'PX4/open_simulator.py &'
			handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

			time.sleep(50)
			c = 'gnome-terminal -- python2 ' + PGFUZZ_HOME + 'PX4/fuzzing.py &'
			handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)


if __name__ == "__main__":
   main(sys.argv[1:])