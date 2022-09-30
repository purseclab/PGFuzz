REBOOT_START = -1
def init():
	global REBOOT_START
	REBOOT_START = 0

def increase():
	global REBOOT_START
	REBOOT_START += 1

def decrease():
	global REBOOT_START
	REBOOT_START -= 1

def boot_get():
	global REBOOT_START
	return REBOOT_START
