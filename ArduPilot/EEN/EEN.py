"""
        Author: Hyungsub Kim
        Date: 08/09/2020
        Name of file: EEN_component.py
        Goal: Testing EEN (Eliminate Environmental Noise) component of PGFUZZ
"""

#!onsusr/bin/env python

import sys, os
from optparse import OptionParser

import time 
import random
import threading

# Tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil, mavwp
from pymavlink import mavextra
from pymavlink import mavexpression
import read_meta_parameter

import time
import re
import math
#------------------------------------------------------------------------------------
# Global variables
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
previous_altitude = 0
current_altitude = 0
current_flight_mode = ""
current_rc_3 = 0
takeoff = 0
drone_status = 0
wp_cnt = 0
lat = 0
lon = 0
alt_distance = 9999
flight_distance = 0
get_target_flight_mode = -1
target_flight_mode = -1
global_distance = 0
previous_global_distance = 0
# if 0: up / 1: down
target_alt_direction = 0
target_throttle = 0
goal_throttle = 0
executing_commands = 0
PARAM_MIN = 1
PARAM_MAX = 10000
required_min_thr = 975

current_roll = 0.0
current_pitch = 0.0
current_heading = 0.0

alt_error = 0.0
roll_error = 0.0
pitch_error = 0.0
heading_error = 0.0
stable_counter = 0

# Variables related to een (eliminate environmental noise)
een_cnt = 0
een_alt = 0.0
een_roll = 0.0
een_pitch = 0.0
een_yaw = 0.0
een_alt_avg = 0.0
een_roll_avg = 0.0
een_pitch_avg = 0.0
een_yaw_avg = 0.0

# Configurations for targeting a specific policy
Target_flight_mode = 2 # 2: ALT_HOLD, 14: FLIP
#------------------------------------------------------------------------------------
def write_log(print_log):
	# Log mutated user command
	mutated_log = open("mutated_log.txt","a")
	mutated_log.write(print_log)
	mutated_log.close()
#------------------------------------------------------------------------------------
def write_een_log(een_log):
	een_print = open("EEN_log.txt", "a")
	een_print.write(een_log)
	een_print.close()

#------------------------------------------------------------------------------------
def verify_real_number(item):
	""" Method to find if an 'item'is real number"""

	item = str(item).strip()
	if not(item):
	    return False
	elif(item.isdigit()):
	    return True
	elif re.match(r"\d+\.*\d*", item) or re.match(r"-\d+\.*\d*", item):
	    return True
	else:
	    return False

#------------------------------------------------------------------------------------
def change_parameter():
	parameter_num = len(read_meta_parameter.param_name)
	selected_param = random.randint(0, parameter_num-1)
	print("# [Change_parameter()] # of parameters: %d, selected params: %s" %(parameter_num, read_meta_parameter.param_name[selected_param]))
	
	no_range = 0
	param_name = read_meta_parameter.param_name[selected_param]
	range_min = read_meta_parameter.param_min[selected_param]
	range_max = read_meta_parameter.param_max[selected_param]
	param_value = 0

	# Step 1. Check whether the selected parameter has an valid range or not
	if range_min == 'X':
		no_range = 1
		param_value = random.randint(PARAM_MIN, PARAM_MAX)
		print("[param] selected params: %s, there is no min of valid range, random param value:%d" % (read_meta_parameter.param_name[selected_param], param_value))

	elif verify_real_number(range_min) == True:
		no_range = 0
		if range_min.isdigit() == True and range_max.isdigit() == True:
			param_value = random.randint(int(range_min), int(range_max))
			print("# [Change_parameter()] selected params: %s, min: %f, max: %f, random digit param value:%d" % (read_meta_parameter.param_name[selected_param], float(range_min), float(range_max), param_value))

		elif range_min.isdigit() == False or range_max.isdigit() == False:
			param_value = random.uniform(float(range_min), float(range_max))
			print("# [Change_parameter()] selected params: %s, min: %f, max: %f, random real param value:%f" % (read_meta_parameter.param_name[selected_param], float(range_min), float(range_max), param_value))
	
	# Step 2. Change the parameter value
        
	# 1) Request parameter
	"""
        master.mav.param_request_read_send(master.target_system, master.target_component, param_name,-1)
        message = master.recv_match(type='PARAM_VALUE', blocking=True)
        message = message.to_dict()
        print('[Change_parameter()] (Before) name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))

        time.sleep(1)
	"""
	global required_min_thr
        if param_name == "FS_THR_VALUE":
                param_value = random.randint(925, 975)
                required_min_thr = param_value
                print("# Required minimum throttle is %d" %param_value)

        # 2) Set parameter value
        master.mav.param_set_send(master.target_system, master.target_component,
                                param_name,
                                param_value,
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # Read ACK
	"""
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('[Change_parameter()] name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
        """

	# Log change parameter values
	print_param = ""
	print_param += param_name
	print_param += " "
	print_param += str(param_value)
	print_param += "\n"

	write_log(print_param)

	time.sleep(3)
	"""
        # Request parameter value to confirm
        master.mav.param_request_read_send(master.target_system, master.target_component, param_name, -1)

        # Print new value in RAM
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('[Change_parameter()] (After) name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
	"""

#------------------------------------------------------------------------------------
#---------------------------- (Start) READ STATES OF AP -----------------------------
def handle_heartbeat(msg):
        global current_flight_mode
        current_flight_mode = mavutil.mode_string_v10(msg)
	
	global drone_status 
	drone_status = msg.system_status
	#print("Drone status: %d, mavlink version: %d" % (drone_status, msg.mavlink_version))

        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        #print("Mode: %s" % current_flight_mode)

#------------------------------------------------------------------------------------
def handle_rc_raw(msg):
	global current_rc_3
	current_rc_3 = msg.chan3_raw

        channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                        msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

#------------------------------------------------------------------------------------
def handle_hud(msg):
	global current_yaw

        hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                                msg.throttle, msg.alt, msg.climb)
        #print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
        #print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

	#print("Alt: %f" %msg.alt)

        global current_altitude
	global previous_altitude
	global current_heading

	previous_altitude = current_altitude
        current_altitude = msg.alt
	
	current_heading = (msg.heading - 359)
#------------------------------------------------------------------------------------
def handle_attitude(msg):

	global current_roll
	global current_pitch

        attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
                                msg.pitchspeed, msg.yawspeed)
        #print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
        #print "%0.6f\t%0.6f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

	current_roll = (msg.roll * 180)/math.pi
	current_pitch = (msg.pitch * 180)/math.pi

#------------------------------------------------------------------------------------
def handle_target(msg):

	global current_altitude
	global current_roll
        global current_pitch
	global current_heading
	global alt_error
	global roll_error
	global pitch_error
	global heading_error
	global stable_counter

	# For een (eliminate environmental noise)
	global een_cnt
	global een_alt
	global een_roll
	global een_pitch
	global een_yaw
	global een_alt_avg
	global een_roll_avg
	global een_pitch_avg
	global een_yaw_avg

	#print("altitude error:%f" %msg.alt_error)
	#msg.alt_error
	"""
	nav_roll	float	deg	Current desired roll
	nav_pitch	float	deg	Current desired pitch
	nav_bearing	int16_t	deg	Current desired heading
	target_bearing	int16_t	deg	Bearing to current waypoint/target
	wp_dist	uint16_t	m	Distance to active waypoint
	alt_error	float	m	Current altitude error
	aspd_error	float	m/s	Current airspeed error
	xtrack_error	float	m	Current crosstrack error on x-y plane
	"""
	reference = (msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.alt_error, msg.aspd_error, msg.xtrack_error)
	#print "\nRF_Roll\tRF_Pitch\tRF_Head\tRF_Alt\tRF_Spd\tRF_XY"
        #print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % reference

	#print("\nRoll error:%f, pitch error:%f, heading error:%f\n" %(abs(msg.nav_roll - current_roll), abs(msg.nav_pitch - current_pitch), abs(msg.nav_bearing - current_heading)))
	
	print("cur_roll: %f, desired_roll:%f" %(current_roll, msg.nav_roll))
	print("cur_pitch: %f, desired_pitch:%f" %(current_pitch, msg.nav_pitch))
	print("cur_yaw: %f, desired_yaw:%f" %(current_heading, msg.nav_bearing))
	
	# For een (eliminate environmental noise)
	een_alt += current_altitude + alt_error
	een_roll += msg.nav_roll
	een_pitch += msg.nav_pitch
	een_yaw += msg.nav_bearing
	een_cnt += 1	

	if een_cnt >= 4:
		een_alt_avg = een_alt / een_cnt
		een_roll_avg  = een_roll / een_cnt
		een_pitch_avg = een_pitch / een_cnt
		een_yaw_avg = een_yaw / een_cnt
		een_cnt = 0
		print("[een] alt: %f, roll: %d, pitch: %f, yaw: %f" %(een_alt_avg, een_roll_avg, een_pitch_avg, een_yaw_avg))		

		# Initialize een variables
		een_alt = 0
		een_roll = 0
		een_pitch = 0
		een_yaw = 0

	# Log for een
        print_een = ""
        print_een += str(time.time())
        print_een += ","
	print_een += str(current_altitude)
        print_een += ","
	print_een += str(current_altitude + alt_error)
        print_een += ","
        print_een += str(een_alt_avg)
        print_een += ","
	print_een += str(current_roll)
        print_een += ","
        print_een += str(een_roll_avg)
        print_een += ","
	print_een += str(current_pitch)
        print_een += ","
        print_een += str(een_pitch_avg)
        print_een += ","
	print_een += str(current_heading)
        print_een += ","
        print_een += str(een_yaw_avg)
        print_een += "\n"
	write_een_log(print_een)


	alt_error += msg.alt_error
	roll_error += abs(msg.nav_roll - current_roll)
	pitch_error += abs(msg.nav_pitch - current_pitch)
	heading_error += abs(msg.nav_bearing - current_heading)
	stable_counter += 1
	
	#print("Desired heading:%f, current heading:%f" %(msg.nav_bearing, current_heading))
	
#------------------------------------------------------------------------------------
def read_loop():   
	
	while True:
		# grab a mavlink message
		msg = master.recv_match(blocking=True)

		# handle the message based on its type
		msg_type = msg.get_type()
		if msg_type == "BAD_DATA":
        		if mavutil.all_printable(msg.data):
		             	sys.stdout.write(msg.data)
		                sys.stdout.flush()
		elif msg_type == "RC_CHANNELS":
			handle_rc_raw(msg)
		elif msg_type == "HEARTBEAT":
		        handle_heartbeat(msg)
		elif msg_type == "VFR_HUD":
		        handle_hud(msg)
		elif msg_type == "ATTITUDE":
	        	handle_attitude(msg)
		elif msg_type == "NAV_CONTROLLER_OUTPUT":
			handle_target(msg)

#---------------------------- (End) READ STATES OF AP -----------------------------
def check_stable():
	
	global current_altitude
        global alt_error
        global roll_error
        global pitch_error
        global heading_error
	global stable_counter
	global current_flight_mode

	while True:
		if stable_counter > 0:
			alt_error = alt_error / stable_counter
			roll_error = roll_error / stable_counter
			pitch_error = pitch_error / stable_counter
			heading_error = heading_error / stable_counter
		
			print("\n### Roll error:%f, pitch error:%f, heading error:%f, cur_alt:%f, alt error:%f, counter:%d, flight mode:%s ###\n" %(roll_error, pitch_error, heading_error, current_altitude, alt_error, stable_counter, current_flight_mode))
			stable_counter = 0
			alt_error = 0
			roll_error = 0
			pitch_error = 0
			heading_error = 0		

		time.sleep(1)

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

#-------------------------------------- Main() --------------------------------------
# Parsing parameters
print("#-----------------------------------------------------------------------------")
read_meta_parameter.parsing_parameter('een_parameters.txt')
print("# Check whether parsing parameters well done or not, received # of params: %d" % len(read_meta_parameter.param_name))
print(read_meta_parameter.param_name)
print("#-----------------------------------------------------------------------------")

# Create the connection
#master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
# Wait a heartbeat before seniding commands
master.wait_heartbeat()

# request data to be sent at the given rate
for i in range(0, 3):
	master.mav.request_data_stream_send(master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)

message = master.recv_match(type='VFR_HUD', blocking=True)

# Choose a mode
mode = 'GUIDED'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

# Check ACK
ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

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
                200) # param7- altitude

ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break



time.sleep(30)


# Maintain mid-position of stick on RC controller 
goal_throttle = 1500
t1 = threading.Thread(target=throttle_th, args=())
t1.daemon = True 
t1.start()

mode_id = master.mode_mapping()['LOITER']
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
# Set default throttle
set_rc_channel_pwm(3, 1500)

time.sleep(3)


t2 = threading.Thread(target=read_loop, args=())
t2.daemon = True
t2.start()

count_main_loop = 0
mutated_log = open("mutated_log.txt","w")
mutated_log.close()

een_log = open("EEN_log.txt", "w")
een_log.close()

t3 = threading.Thread(target=check_stable, args=())
t3.daemon = True
t3.start()


# Main loop
while True:
	change_parameter()		
	time.sleep(5)

print("-------------------- Fuzzing End --------------------")
