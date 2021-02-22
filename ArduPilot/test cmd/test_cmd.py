"""
        Author: Hyungsub Kim
        Date: 05/26/2020
        Name of file: input_generator.py
        Goal: Implementing user commands
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
#from pymavlink import mavtest
#import read_meta_parameter

import time
import re
import math
#------------------------------------------------------------------------------------
# Global variables
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
home_altitude = 0
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
#------------------------------------------------------------------------------------
def write_log(print_log):
	# Log mutated user command
	mutated_log = open("mutated_log.txt","a")
	mutated_log.write(print_log)
	mutated_log.close()

#------------------------------------------------------------------------------------
def re_launch():

	global goal_throttle
	global home_altitude
	global current_altitude
	global current_flight_mode

	print("#-----------------------------------------------------------------------------")
	print("#-----------------------------------------------------------------------------")
	print("#------------------------- RE-LAUNCH the vehicle -----------------------------")
	print("#-----------------------------------------------------------------------------")
	print("#-----------------------------------------------------------------------------")

	# Step 1. land the vehicle
	master.mav.set_mode_send(
	    master.target_system,
	    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
	    9)
	
	# Wait for finishing the landing
	while True:
		if current_flight_mode == "LAND":
			break
		time.sleep(0.2)

	home_altitude = current_altitude

	time.sleep(5)

	# Initializing RC channels
	global required_min_thr
	goal_throttle = required_min_thr + 1
	set_rc_channel_pwm(1, 1500)
	set_rc_channel_pwm(2, 1500)
	set_rc_channel_pwm(4, 1500)

	time.sleep(3)

	# Step 2. re-take off the vehicle
	master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4)

        # Wait for finishing the landing
        while True:
                if current_flight_mode == "GUIDED":
                        break
                time.sleep(0.2)

	time.sleep(3)

	# Arming
	master.mav.command_long_send(
    		master.target_system,
		master.target_component,
	    	mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    		0,
    		1, 0, 0, 0, 0, 0, 0)

	
	time.sleep(3)

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

	time.sleep(18)
	goal_throttle = 1500

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
		print("# [Change_parameter()] selected params: %s, there is no min of valid range, random param value:%d" % (read_meta_parameter.param_name[selected_param], param_value))

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
	global required_min_thr
	if param_name == "FS_THR_VALUE":
		required_min_thr = param_value
		print("# Required minimum throttle is %d" %param_value)

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
def change_wind_dir(value):
	"""
        # Request parameter
        master.mav.param_request_read_send(master.target_system, master.target_component, 'SIM_WIND_DIR',-1)
        message = master.recv_match(type='PARAM_VALUE', blocking=True)
        message = message.to_dict()
        print('(Before) name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
	
        time.sleep(1)
	"""
        # Set parameter value
        if value == 1:
                wind_dir = random.randint(1, 360)
        elif value == 0:
                wind_dir = 180

        master.mav.param_set_send(master.target_system, master.target_component,
                                'SIM_WIND_DIR',
                                wind_dir,
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
	# Read ACK
	"""
	message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
	"""

        time.sleep(3)
	"""
        # Request parameter value to confirm
        master.mav.param_request_read_send(master.target_system, master.target_component,'SIM_WIND_DIR',-1)

        # Print new value in RAM
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('(After) name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
	"""
        # Log mutated user command
        print_cmd = ""
        print_cmd += "wind_dir"
        print_cmd += " "
        print_cmd += str(wind_dir)
        print_cmd += "\n"
        write_log(print_cmd)

#------------------------------------------------------------------------------------
def change_wind(value):
	"""
	# Request parameter
	master.mav.param_request_read_send(master.target_system, master.target_component, 'SIM_WIND_SPD',-1)
	message = master.recv_match(type='PARAM_VALUE', blocking=True)
	message = message.to_dict()
	print('(Before) name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))

	time.sleep(1)
	"""
	# Set parameter value
	if value == 1:
		rand_wind = random.randint(1, 2)
	elif value == 0:
		rand_wind = 0

	master.mav.param_set_send(master.target_system, master.target_component,
                          	'SIM_WIND_SPD',
                           	rand_wind,
                           	mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
	# Read ACK
	"""
	message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
	print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
	"""
	time.sleep(3)
	"""
	# Request parameter value to confirm
	master.mav.param_request_read_send(master.target_system, master.target_component,'SIM_WIND_SPD',-1)

	# Print new value in RAM
	message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
	print('(After) name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
	"""
        # Log mutated user command
        print_cmd = ""
        print_cmd += "wind"
        print_cmd += " "
        print_cmd += str(rand_wind)
        print_cmd += "\n"
        write_log(print_cmd)

#------------------------------------------------------------------------------------
#---------------------------- (Start) READ STATES OF AP -----------------------------
def handle_heartbeat(msg):
        global current_flight_mode
        current_flight_mode = mavutil.mode_string_v10(msg)
	
	global drone_status 
	drone_status = msg.system_status
	print("Drone status: %d, mavlink version: %d" % (drone_status, msg.mavlink_version))

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
        hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                                msg.throttle, msg.alt, msg.climb)
        #print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
        #print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

	print("[Status] Alt: %f" %msg.alt)

        global current_altitude
	global previous_altitude

	previous_altitude = current_altitude
        current_altitude = msg.alt

#------------------------------------------------------------------------------------
def handle_attitude(msg):
        attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
                                msg.pitchspeed, msg.yawspeed)
        #print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
        #print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

#------------------------------------------------------------------------------------
def handle_target(msg):
	#print("altitude error:%f" %msg.alt_error)
	msg.alt_error

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
#------------------------------------------------------------------------------------
def calculate_distance():
	
	global alt_distance
	global flight_distance
	global current_altitude
	global current_flight_mode
	global global_distance
	global previous_global_distance
	# if 0: up / 1: down
	global target_alt_direction
	global target_flight_mode
	global HUD_LOCK
	global get_target_flight_mode

	temp_alt_distance = alt_distance	
	#previous_altitude = current_altitude

	#HUD_LOCK = 1

	#msg = master.recv_match(type='VFR_HUD', blocking=True)
        #handle_hud(msg)

	#HUD_LOCK = 0
	# 584: 0m / 594: 10m
	alt_distance = (current_altitude - 594)/594
	print("### Current altitude: %f, altitude distance: %f ###" %(current_altitude, alt_distance))
	
        if get_target_flight_mode == 14:
                flight_distance = 1
        elif get_target_flight_mode != 14:
                flight_distance = -1

	"""
	if current_flight_mode == "FLIP":
		flight_distance = 1
	elif current_flight_mode != "FLIP":
		flight_distance = -1
	"""
	print("### Current flight mode:%s, flight mode distance: %f ###" %(current_flight_mode, flight_distance))

	minimum = min(alt_distance, flight_distance)
	previous_global_distance = global_distance
	global_distance = minimum
	print("### Global distance: %f ###" %global_distance)

	# When the distance is increasing and direction is upward
	"""
	if (abs(temp_alt_distance) < abs(alt_distance)) and (target_alt_direction == 0):
		target_alt_direction = 1
	elif (abs(temp_alt_distance) < abs(alt_distance)) and (target_alt_direction == 1):
		target_alt_direction = 0
	"""
	
	# if 0: up / 1: down
	# 1) RC: up, Alt: up, Dist: up --> Action: down direction
        if (previous_altitude < current_altitude and abs(temp_alt_distance) < abs(alt_distance)) and target_throttle > 1500:
                target_alt_direction = 1
        # 2) RC: up, Alt: down, Dist: up --> Action: up direction
        elif (previous_altitude > current_altitude and abs(temp_alt_distance) < abs(alt_distance)) and target_throttle > 1500:
                target_alt_direction = 0
	# 3) RC: up, Alt: down, Dist: down --> Action: down direction
	elif (previous_altitude > current_altitude and abs(temp_alt_distance) > abs(alt_distance)) and target_throttle > 1500:
                target_alt_direction = 1
	# 4) RC: up, Alt: up, Dist: down --> Action: up direction
        elif (previous_altitude < current_altitude and abs(temp_alt_distance) > abs(alt_distance)) and target_throttle > 1500:
                target_alt_direction = 0	
	# 5) RC: down, Alt: up, Dist: up --> Action: down direction
        elif (previous_altitude < current_altitude and abs(temp_alt_distance) < abs(alt_distance)) and target_throttle < 1500:
                target_alt_direction = 1
	# 6) RC: down, Alt: up, Dist: down --> Action: up direction
        elif (previous_altitude < current_altitude and abs(temp_alt_distance) > abs(alt_distance)) and target_throttle < 1500:
                target_alt_direction = 0
	# 7) RC: down, Alt: down, Dist: down --> Action: down direction
        elif (previous_altitude > current_altitude and abs(temp_alt_distance) > abs(alt_distance)) and target_throttle < 1500:
                target_alt_direction = 1
	# 8) RC: down, Alt: down, Dist: up --> Action: up direction
        elif (previous_altitude > current_altitude and abs(temp_alt_distance) < abs(alt_distance)) and target_throttle < 1500:
                target_alt_direction = 0


	"""
	if (abs(temp_alt_distance) < abs(alt_distance)) and target_throttle <= 1500:
		# Target direction = up
		target_alt_direction = 0
	# Distance is increased or same && throttle >= 1500
	elif (abs(temp_alt_distance) <= abs(alt_distance)) and target_throttle >= 1500:
		target_alt_direction = 1
	"""
	"""
	elif (abs(temp_alt_distance) > abs(alt_distance)) and target_throttle <= 1500:
                target_alt_direction = 0
	elif (abs(temp_alt_distance) >= abs(alt_distance)) and target_throttle >= 1500:
                target_alt_direction = 1
	"""

	
	if target_alt_direction == 0:
		print("### target altitude direction is upward")
	elif target_alt_direction == 1:
		print("### target altitude direction is downward")
		
	
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
def randomly_pick_up_cmd(mandatory):
	
	global executing_commands
	#num = random.randint(1,8)
	
	# Test
	"""	
	if num % 2 == 0:
		num = 1
	elif num % 2 == 1:
		num = 3
	"""
	#print("# Random number: %d" % num)

	# 590: 6m
	#global current_altitude
	#if current_altitude < 589:
		#set_rc_channel_pwm(3, 1500)

	#if num == 1 and current_altitude >= 592:
	for num in range(1, mandatory):

		# rc_channels_override_send	
		if num == 1:
			global target_alt_direction

			if target_alt_direction == 0: # up direction
				print("[1] Random user command: change throttle (up direction)")
				rand_value = random.randint(1600, 1650)

			elif target_alt_direction == 1: # down direction
				print("[1] Random user command: change throttle (down direction)")
				rand_value = random.randint(1350, 1400)		

			print("# rand_value for throttle: %d" % rand_value)

			global target_throttle
			global goal_throttle
			
			# --- Start to change throttle value ---
			#t0 = time.clock()
			target_throttle = rand_value
			goal_throttle = target_throttle
	
			"""
			set_rc_channel_pwm(3, rand_value)
			time.sleep(1)
			"""
			global current_rc_3

			while(True):
				if current_rc_3 == rand_value:
					print("Command is completed (target RC: %d)" % current_rc_3)
					break

				time.sleep(0.1)

			time.sleep(3)
					
		        # Log mutated user command
		        print_cmd = ""
		        print_cmd += "throttle"
		        print_cmd += " "
		        print_cmd += str(goal_throttle)
		        print_cmd += "\n"
			write_log(print_cmd)

			goal_throttle = 1500

			# --- End to change throttle value ---
                        #t1 = time.clock() - t0
                        #print("Throttle - time elapsed: ", t1) # CPU seconds elapsed (floating point)

			#set_rc_channel_pwm(3, 1500)

		#-------------------------------------------------------				
		# MAV_CMD_NAV_TAKEOFF
		elif num == 999:
			print("[2] Random user command: takeoff")
			# 583.989990 (absolute altitude) == 0m (relative altitude)
			global takeoff
	
			if takeoff == 1:
				print("ArduPilot already conducted takeoff")
				continue

			global drone_status

                        # Conduct takeoff when the drone is standby status
                        if drone_status != 3:
                                print("Drone is not standby status")
                                continue


			rand_alt = random.randint(1, 20)
			print("rand_alt %d" % rand_alt)

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
	                rand_alt) # param7- altitude

			"""
			ack = False
			while not ack:
	    			# Wait for ACK command
	    			ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
	    			ack_msg = ack_msg.to_dict()

	    			print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
	   			break
			"""
			while(True):
				print("cur_alt: %f, target_alt: %f" %(current_altitude, (home_altitude + rand_alt)))

	                        if current_altitude >= (home_altitude + rand_alt):
					takeoff = 1
                	                print("Command is completed (target altitude: %d)" % current_altitude)
                        	        break
			
				time.sleep(0.1)

			# Log mutated user command
			print_cmd = ""
			print_cmd += "takeoff"
			print_cmd += " "
			print_cmd += str(rand_alt)
			print_cmd += "\n"
			write_log(print_cmd)
		#-------------------------------------------------------
		# Change flight mode
		elif num == 3:
			global get_target_flight_mode
			
			print("[3] Random user command: change flight mode")
			
			if get_target_flight_mode == -1:
	        		rand_fligh_mode = random.randint(1, 20)
			elif get_target_flight_mode != -1:
				rand_fligh_mode = get_target_flight_mode	

			# Test
			"""		
			if rand_fligh_mode % 3 == 0:
				rand_fligh_mode = 14
			elif rand_fligh_mode % 3 == 1:
				#if target_flight_mode != -1:
				#rand_fligh_mode = target_flight_mode
				rand_fligh_mode = 14
			elif rand_fligh_mode % 3 == 2:
				rand_fligh_mode = 2
			"""
			# For developing
			rand_fligh_mode = 14

			if rand_fligh_mode == 14:
				get_target_flight_mode = 14
				# To trigger the Flip mode, it requires AP to be ALT_HOLD (2) mode
				master.mav.set_mode_send(
                                master.target_system,
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                2)
				
				time.sleep(3)
				
	                       	# Log mutated user command
	                        print_cmd = ""
	                        print_cmd += "mode"
	                        print_cmd += " "
	                        print_cmd += str(2)
	                        print_cmd += "\n"
	                        write_log(print_cmd)

			# if Flip mode is triggered at least one time, keep trigger the Flip mode
			if flight_distance == 1:
				rand_fligh_mode = 14		
			
			# --- Start to change flight mode ---
                        #t0 = time.clock()
		
			master.mav.set_mode_send(
	            		master.target_system,
	            		mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
	            		rand_fligh_mode)
	        	print("Selected mode id: %d" % rand_fligh_mode)
			"""
			# Check ACK
			ack = False
			while not ack:
	    			# Wait for ACK command
	    			ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
	    			ack_msg = ack_msg.to_dict()
	    			break
			"""
			time.sleep(2)

			# Log mutated user command
                        print_cmd = ""
                        print_cmd += "mode"
                        print_cmd += " "
                        print_cmd += str(rand_fligh_mode)
                        print_cmd += "\n"
                        write_log(print_cmd)		

			# --- End to change flight mode ---
                        #t1 = time.clock() - t0
                        #print("flight mode - time elapsed: ", t1) # CPU seconds elapsed (floating point)

		#-------------------------------------------------------
		elif num == 4:
			print("[4] Random user command: change roll angle")
			# Change roll 
			rand_roll = random.randint(1400, 1600)
			print("Target roll value: %d" %rand_roll)
			set_rc_channel_pwm(1, rand_roll)		
			time.sleep(1.5)
			set_rc_channel_pwm(1, 1500)
			#time.sleep(1)
			
			# Log mutated user command
                        print_cmd = ""
                        print_cmd += "roll"
                        print_cmd += " "
                        print_cmd += str(rand_roll)
                        print_cmd += "\n"
                        write_log(print_cmd)
			

		# Call distance metric function
                calculate_distance()


	for x in range(1, 2):
		num = random.randint(4,9)
		# Test
		#num = 9
		#-------------------------------------------------------
		# Change wind
		if num == 4:	
			change_wind(1)
		#-------------------------------------------------------
		# Change wind direction
		elif num == 5:
			change_wind_dir(1)
		#-------------------------------------------------------
		elif num == 6:
			print("# Random user command: add a waypoint")
			"""
			master.mav.command_long_send(
	                master.target_system,  # target_system
	                master.target_component, # target_component
	                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # command
	                0, # confirmation
	                0, # param1
        	        0, # param2
	                0, # param3
	                0, # param4
	                0, # param5
	                0, # param6
	                60) # param7- altitude
			"""
			"""		
			master.mav.mission_item_send(master.target_system,
	                                      master.target_component,
	                                      1,
	                                      3,
	                                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
	                                      3, 1, 0, 0, 0, 0,
	                                      0, 0, 60)
			"""
			wp_alt = random.randint(50, 100)
			global wp_cnt, lat, lon
			lat = -35+random.uniform(0, 2)
			lon = 149+random.uniform(0, 2)
			master.mav.mission_item_send(master.target_system,master.target_component,
						     wp_cnt,
						     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
						     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
						     2,0,0,0,0,0,
						     lat,lon,wp_alt)
			wp_cnt = wp_cnt + 1
	    		print("Sent change waypoint command (lat:%f, lon:%f, alt:%f)" % (lat, lon, wp_alt))
	
			# In order to prevent crash the drone (04/11/2020)
			time.sleep(2)
			set_rc_channel_pwm(3, 1600)
			time.sleep(2)

                        # Log mutated user command
                        print_cmd = ""
                        print_cmd += "add_waypoint"
                        print_cmd += " "
                        print_cmd += str(lat)
			print_cmd += " "
			print_cmd += str(lon)
			print_cmd += " "
			print_cmd += str(wp_alt)
                        print_cmd += "\n"
                        write_log(print_cmd)

		#-------------------------------------------------------
        	elif num == 7:
			print("# Random user command: change pitch angle")
	                # Change pitch
	                rand_pitch = random.randint(1000, 2000)
	                print("Target pitch value: %d" %rand_pitch)
	                set_rc_channel_pwm(2, rand_pitch)
	                time.sleep(1.5)
	                set_rc_channel_pwm(2, 1500)

                        # Log mutated user command
                        print_cmd = ""
                        print_cmd += "pitch"
                        print_cmd += " "
                        print_cmd += str(rand_pitch)
                        print_cmd += "\n"
                        write_log(print_cmd)

		#-------------------------------------------------------
	        elif num == 8:
	                print("# Random user command: change yaw angle")
	                # Change yaw
	                rand_yaw = random.randint(1000, 2000)
	                print("Target yaw value: %d" %rand_yaw)
	                set_rc_channel_pwm(4, rand_yaw)
	                time.sleep(1.5)
	                set_rc_channel_pwm(4, 1500)

			# Log mutated user command
                        print_cmd = ""
                        print_cmd += "yaw"
                        print_cmd += " "
                        print_cmd += str(rand_yaw)
                        print_cmd += "\n"
                        write_log(print_cmd)

		#-------------------------------------------------------
		elif num == 9:
			print("# Random user command: Loiter around this waypoint an unlimited amount of time");		
			nav_loiter_unlim_alt = random.randint(50, 100)
                        nav_loiter_unlim_lat = -35+random.uniform(0, 2)
                        nav_loiter_unlim_lon = 149+random.uniform(0, 2)

			master.mav.command_long_send(master.target_system, master.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, 5, 0, nav_loiter_unlim_lat, nav_loiter_unlim_lon, nav_loiter_unlim_alt)			
			
			# Log mutated user command
                        print_cmd = ""
                        print_cmd += "nav_loiter_unlim"
			print_cmd += " "
                        print_cmd += str(nav_loiter_unlim_lat)
                        print_cmd += " "
                        print_cmd += str(nav_loiter_unlim_lon)
                        print_cmd += " "
                        print_cmd += str(nav_loiter_unlim_alt)
                        print_cmd += "\n"
                        write_log(print_cmd)
                        
			
		# Call distance metric function
		calculate_distance()

	executing_commands = 0

#------------------------------------------------------------------------------------

# Create the connection
#master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
# Wait a heartbeat before seniding commands
master.wait_heartbeat()

# request data to be sent at the given rate
for i in range(0, 3):
	master.mav.request_data_stream_send(master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)

message = master.recv_match(type='VFR_HUD', blocking=True)
home_altitude = message.alt
print("home_altitude: %f" %home_altitude)

#master.mav.request_data_stream_send(master.target_system, master.target_component,
#                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 4, 1)

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
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
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

# Set wind as default
change_wind(0)
change_wind_dir(0)

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
"""
master.mav.mission_item_send(master.target_system,
                                      master.target_component,
                                      0,
                                      3,
                                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                      3, 1, 0, 0, 0, 0,
                                      0, 0, 60)

master.mav.command_long_send(
                master.target_system,  # target_system
                master.target_component, # target_component
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # command
                0, # confirmation
                0, # param1
                0, # param2
                30, # param3
                20, # param4
                0, # param5
                0, # param6
                30) # param7- altitude

"""

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

wp_add_alt = random.randint(50, 100)

lat = -35+random.uniform(0, 2)
lon = 148+random.uniform(0, 2)
master.mav.mission_item_send(master.target_system,master.target_component,
                             wp_cnt,
                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                             2,0,0,0,0,0,
                             lat,lon,wp_add_alt)
wp_cnt = wp_cnt + 1
print("Sent change waypoint command (lat:%f, lon:%f, alt:%f)" % (lat, lon, wp_add_alt))

time.sleep(5)

# Maintain mid-position of stick on RC controller 
goal_throttle = 1500
t1 = threading.Thread(target=throttle_th, args=())
t1.daemon = True 
t1.start()

"""
# Set default throttle
set_rc_channel_pwm(3, 1500)
time.sleep(1)
"""

mode_id = master.mode_mapping()['AUTO']
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
# Set default throttle
set_rc_channel_pwm(3, 1500)
set_rc_channel_pwm(2, 1200)
time.sleep(3)

t2 = threading.Thread(target=read_loop, args=())
t2.daemon = True
t2.start()

"""
master.mav.command_long_send(
            master.target_system,  # target_system
            master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            0,
            2,
            0, 0, 0, 0, 0, 0)
"""

"""
alt = random.randint(50, 100)
lat = -35+random.uniform(0, 2)
lon = 149+random.uniform(0, 2)

master.mav.command_long_send(
            master.target_system,  # target_system
            master.target_component, # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            1,
            0, 0, 0, 0, 0, 0)
"""
"""
master.mav.command_long_send(
	master.target_system,  # target_system
        master.target_component, # target_component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1,
        2, 2000, 0, 0, 0, 0)
"""
"""
master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component, # target_component
        mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
        0,
        0,
        0, 1, 3, 1, 0, 0)
"""
"""
master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component, # target_component
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
        0,
        5,
        0, 0, 0, 0, 0, 0)
"""
"""
master.mav.command_long_send(
	master.target_system, 
	master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS, 
	0, 0, 5, 0, 0, 0, 0, 0, 0)
"""

mavtest.generate_outputs(master.mav)

time.sleep(3)



while True:
	time.sleep(1)
