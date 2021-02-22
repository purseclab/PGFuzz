import random

"""
f = open("./policies/chute/cmds.txt", "r")
f_lines = f.readlines()

print(f_lines[0].replace("\n", ""))
print(f_lines[1].replace("\n", ""))
print(f_lines[2].replace("\n", ""))
"""

"""
lines = ""
for line in open("guidance_log.txt", 'r').readlines():
	lines += line
	if "RC3" in line:
		#row = line.rstrip().split(' ')
		#print("# {} {} {}".format(row[0], row[1], row[2]))
		print("find it!")
		lines = lines.replace(line, "Fliht_Mode_new\n")

print(lines)
"""
"""
def func1():

	return 1234

result = func1()
print(result)
"""

"""
for i in range(10):
	decision = None
	decision = random.choice([True, False])
	print(decision)
		
"""

"""
file_name = ""

f1 = open("mutated_log.txt","r")
lines = f1.readlines()

file_name += "mutated_log"
file_name += str(2)
file_name += ".txt"
f2 = open(file_name,"w")
f2.writelines(lines)

f1.close()
f2.close()
"""

"""
for i in range(3):
	print(i)
"""

"""
lat = 1491652368
print("%f"%lat)
lat = lat / 1000
lat = lat * 1000
print("%f"%lat)
"""
"""
j = "severity:2	PreArm: Check ANGLE_MAX"

if "PreArm: Check" in j:
	print("@@@@@@@@@@@")
"""
for i in range (1, 3):
	print(i)
