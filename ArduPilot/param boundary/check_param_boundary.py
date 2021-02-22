"""
        Author: Hyungsub Kim
        Date: 05/24/2020
        Name of file: input_generator.py
        Goal: Profiling flight mode on flight control software
"""

#!onsusr/bin/env python

import sys, os
from optparse import OptionParser

# Tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil, mavwp
from pymavlink import mavextra
from pymavlink import mavexpression

import time
import threading

#------------------------------------------------------------------------------------
# Global variables
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Reference: https://ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html
rows, cols = (25, 25)
mode_map=[]
mode_i = 0
mode_j = 0
goal_throttle = 0
paramsName = []
Min_param = -99999999
Max_param = 99999999
#------------------------------------------------------------------------------------
# For testing
def convert_mode():
        
        global mode_i
        global mode_j

        for mode_j in range(0, 25):
        	if mode_i == mode_j:
                	continue

                change_mode(mode_j, True)       # Try to change a specific flight mode
                change_mode(mode_i, False)      # Roll-back to the original flight mode

"""
def convert_mode():
	
	global mode_i
	global mode_j

	for mode_i in range(0, 25):

                if mode_i == 8 or mode_i == 10 or mode_i == 12:
                        continue

                for mode_j in range(0, 25):
                        if mode_i == mode_j:
                                continue

                        change_mode(mode_j, True)       
                        change_mode(mode_i, False)      
"""

#------------------------------------------------------------------------------------
def profiling_mode():

	convert_mode()

	# Finally print mode map
	for i in range(0, 25):
		print(mode_map[i])

#------------------------------------------------------------------------------------
def read_params():

	global paramsName
	
	for line in open('default_params.txt', 'r').readlines():
		row = line.rstrip().split(' ')
		paramsName.append(row[0])

	# For debug
	#print(paramsName)
		
#------------------------------------------------------------------------------------
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm

        #global master

        master.mav.rc_channels_override_send(
            master.target_system,                # target_system
            master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.

#------------------------------------------------------------------------------------
def throttle_th():

	global goal_throttle

        while True:
                set_rc_channel_pwm(3, goal_throttle)
                time.sleep(0.2)

#------------------------------------------------------------------------------------
def change_mode(mode_num, log):
	master.mav.set_mode_send(
	    master.target_system,
	    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
	    mode_num)

	# Check ACK
	ack = False
	global mode_i
	global mode_j

	while not ack:
		"""
		# Wait for ACK command
	    	ack_command = master.recv_match(type='COMMAND_ACK', blocking=True)
		ack_msg = ack_command.to_dict()

		# Check if command in the same in `set_mode`
		if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
	        	continue
		"""
		time.sleep(0.05)
	    	# Print the ACK result !
	    	#print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
		"""
		# result:0 <-- successful / result: 3 <-- fail
		if ack_command.result == 0:
			#print("Sucessful")
			if log == True:
				mode_map[mode_i][mode_j] = 1
		elif ack_command.result != 0:
			#print("Fail")
			if log == True:
				mode_map[mode_i][mode_j] = 2
		"""
	    	break

#-------------------------- (Start) Step1. Flying a drone -------------------------- 

# Initialize mode map 
for i in range(cols): 
    col = [] 
    for j in range(rows): 
        col.append(0) 
    mode_map.append(col) 

# Wait a heartbeat before seniding commands
master.wait_heartbeat()

# Choose a mode
mode = 'GUIDED'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]

# Set new mode
change_mode(mode_id, False)

# Arming
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

time.sleep(1)

# Taking off
master.mav.command_long_send(
                master.target_system,  # target_system
                master.target_component, # target_component
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                0, # confirmation
                0, # param1
                0, # param2
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                50) # param7- altitude

ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

time.sleep(18)


# Maintain mid-position of stick on RC controller 
goal_throttle = 1560
t1 = threading.Thread(target=throttle_th, args=())
t1.daemon = True 
t1.start()

# Read default parameter list
read_params()

#for i in range(1230, 1308):
for i in range(0, 1308):

	# Skipping already found bugs	
	#if paramsName[i] == 'AHRS_TRIM_Z' or paramsName[i] == 'ANGLE_MAX' or paramsName[i] == 'BATT_AMP_OFFSET' or paramsName[i] == 'BATT_AMP_PERVLT' or paramsName[i] == 'BATT_VOLT_MULT' or paramsName[i] == 'COMPASS_DIA2_X':
	#	continue

	print("%s:%d" % (paramsName[i], Min_param))
	master.mav.param_set_send(master.target_system, master.target_component,
                                paramsName[i],
                                Min_param,
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

	time.sleep(1.5)
	convert_mode()
	
	print("%s:%d" % (paramsName[i], Max_param))
	master.mav.param_set_send(master.target_system, master.target_component,
                                paramsName[i],
                                Max_param,
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
	time.sleep(1.5)
	convert_mode()
