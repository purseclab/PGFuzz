"""
	Author: Hyungsub Kim
	Date: 05/20/2020
	Name of file: read_inputs.py
	Goal: Parsing a meta file for inputs
"""
#!/usr/bin/python

param_name = []
param_reboot = []
param_default = []
param_min = []
param_max = []
param_units = []

cmd_name = []
cmd_number = []

env_name = []

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
def parsing_parameter(filepath):

	print('##### (Start) Read a meta file for parameters #####')
	cnt = 0

	#filepath = 'meta_parameters.txt'
	for line in open(filepath, 'r').readlines():

		row = line.rstrip().split(',')
		param_name.append(row[0])
		param_reboot.append(row[1])	
		param_default.append(row[2])
		param_min.append(row[3])
		param_max.append(row[4])
		param_units.append(row[5])
		# row[0]: name of parameter, row[1]: reboot required, row[2]: Default value, row[3]: Valid range min, row[4]: Valid range max, row[5] Units
		cnt += 1
		print("# {} {} {} {} {} {} {}".format(cnt, row[0], row[1], row[2], row[3], row[4], row[5]))


	print("##### The name of parameters #####");
	print(param_name)
	print(param_reboot)
	print(param_default)
	print(param_min)
	print(param_max)
	print(param_units)

	print('##### (End) Read a meta file for parameters #####')
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

def parsing_command(filepath):

	print('##### (Start) Read a meta file for user commands #####')
	cnt = 0

	for line in open(filepath, 'r').readlines():
		#row = line.replace("\n", "")
		row = line.rstrip().split(',')
		cmd_name.append(row[0])
		cmd_number.append(row[1])
		cnt += 1

	print("##### The name of user commands #####");
	print(cmd_name)
	print(cmd_number)
	print('##### (End) Read a meta file for user commands #####')
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

def parsing_env(filepath):

	print('##### (Start) Read a meta file for environmental factors #####')
	cnt = 0

	for line in open(filepath, 'r').readlines():
		row = line.replace("\n", "")
		env_name.append(row)
		cnt += 1

	print("##### The name of environmental factors #####");
	print(env_name)
	print('##### (End) Read a meta file for environmental factors #####')
