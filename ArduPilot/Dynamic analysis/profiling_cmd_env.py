"""
        Author: Hyungsub Kim
        Date: 05/26/2020
        Name of file: profiling_cmd_env.py
        Goal: Mapping a user command / environmental factor to each state (term)

	- This version supports total 34 states
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
# -------------------------
avg_start = 0
avg_airspeed = []
avg_groundspeed = []
avg_heading = []
avg_throttle = []
avg_alt = []
avg_climb = []
hud_cnt = 0

avg_roll = []
avg_pitch = []
avg_yaw = []
avg_rollspeed = []
avg_pitchspeed = []
avg_yawspeed = []
atti_cnt = 0

position_cnt = 0
avg_lat = []
avg_lon = []

avg_rc1 = []
avg_rc2 = []
avg_rc3 = []
avg_rc4 = []

avg_refer_roll = []
avg_refer_pitch = []
avg_refer_yaw = []
avg_refer_aspd = []
avg_refer_alt = []

current_aspd = 0
system_time = 0

Parachute = 0
GPS_status = 1
Accel_status = 1
Gyro_status = 1
Baro_status = 1
PreArm_error = 0
mission_cnt = 0

SD_baseline = []
SD_target_input = []

cmd_name = []
cmd_number = []
env_name = []
target_param = ""
target_param_ready = 0
target_param_value = 0
start_profiling = 0

avg_num_GPS = [] 
avg_alt_GPS = []
avg_vertical_speed = []

# Configuration
Precondition_path = "preconditions.txt"
Measuring_duration = 1
Measuring_iteration = 3
minimum_sd = 0.00001
number_of_states = 34
#------------------------------------------------------------------------------------
def set_preconditions(filepath):

        for line in open(filepath, 'r').readlines():
                row = line.rstrip().split(' ')

                master.mav.param_set_send(master.target_system, master.target_component,
                                row[0],
                                float(row[1]),
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                time.sleep(1)

                print("[Set_preconditions] %s = %s" %(row[0], row[1]) )

#------------------------------------------------------------------------------------
def parsing_env(filepath):

	global env_name

	print('##### (Start) Read a meta file for environmental factors #####')
	cnt = 0

	for line in open(filepath, 'r').readlines():
		row = line.replace("\n", "")
		env_name.append(row)
		cnt += 1

	print("##### The name of environmental factors #####");
	print(env_name)
	print('##### (End) Read a meta file for environmental factors #####')
#------------------------------------------------------------------------------------
def parsing_command(filepath):

	global cmd_name
	global cmd_number

	print('##### (Start) Read a meta file for user commands #####')
	cnt = 0

	for line in open(filepath, 'r').readlines():
		#row = line.replace("\n", "")
		row = line.rstrip().split(',')
		cmd_name.append(row[0])
		cmd_number.append(int(row[1]))
		cnt += 1

	print("##### The name of user commands #####");
	print(cmd_name)
	print(cmd_number)
	print('##### (End) Read a meta file for user commands #####')

#------------------------------------------------------------------------------------
def Standard_deviation(list):
	# Standard deviation of list 
	# Using sum() + list comprehension 

	# Prevent division by zero
	if len(list) > 0:
		mean = sum(list) / len(list) 
		variance = sum([((x - mean) ** 2) for x in list]) / len(list) 
		res = variance ** 0.5
	else:
		res = 0
	
	return res
#------------------------------------------------------------------------------------
#---------------------------- (Start) READ STATES OF AP -----------------------------
def handle_heartbeat(msg):
        global current_flight_mode
        current_flight_mode = mavutil.mode_string_v10(msg)
	
	global drone_status 
	drone_status = msg.system_status
#	print("Drone status: %d, mavlink version: %d" % (drone_status, msg.mavlink_version))

        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        #print("Mode: %s" % current_flight_mode)

#------------------------------------------------------------------------------------
def handle_rc_raw(msg):
	global current_rc_3
	global avg_rc1
	global avg_rc2
	global avg_rc3
	global avg_rc4

	current_rc_3 = msg.chan3_raw

        channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                        msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

	#print("[DEBUG] RC1:%f, RC2:%f, RC3:%f, RC4:%f" %(msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw) )
	# Utilizing a time window
        if avg_start == 1:
		avg_rc1.append(msg.chan1_raw)
		avg_rc2.append(msg.chan2_raw)
		avg_rc3.append(msg.chan3_raw)
		avg_rc4.append(msg.chan4_raw)

#------------------------------------------------------------------------------------
def handle_hud(msg):
        hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                                msg.throttle, msg.alt, msg.climb)
        #print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
        #print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

	#print("[Status] Alt: %f" %msg.alt)

        global current_altitude
	global previous_altitude
	global avg_airspeed
	global avg_groundspeed
	global avg_heading
	global avg_throttle
	global avg_alt
	global avg_climb
	global hud_cnt
	global avg_start
	global current_aspd

	# Utilizing a time window
	if avg_start == 1:
		# Store average of hud data
		hud_cnt += 1
		avg_airspeed.append(msg.airspeed)
		avg_groundspeed.append(msg.groundspeed)
		avg_heading.append(msg.heading)
		avg_throttle.append(msg.throttle)
		avg_alt.append(msg.alt)
		avg_climb.append(msg.climb)
	
	previous_altitude = current_altitude
        current_altitude = msg.alt
	current_aspd = msg.airspeed

#------------------------------------------------------------------------------------
def handle_attitude(msg):
        attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
                                msg.pitchspeed, msg.yawspeed)

	global avg_start
	global atti_cnt
	global avg_roll
        global avg_pitch
        global avg_yaw
        global avg_rollspeed
        global avg_pitchspeed
        global avg_yawspeed

	if avg_start == 1:
                # Store average of hud data
                atti_cnt += 1
                avg_roll.append(msg.roll)
                avg_pitch.append(msg.pitch)
                avg_yaw.append(msg.yaw)
                avg_rollspeed.append(msg.rollspeed)
                avg_pitchspeed.append(msg.pitchspeed)
                avg_yawspeed.append(msg.yawspeed)

#        print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
#        print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

#------------------------------------------------------------------------------------
def handle_target(msg):

	global avg_refer_roll
        global avg_refer_pitch
        global avg_refer_yaw
        global avg_refer_aspd
        global avg_refer_alt
	global current_altitude
	global current_aspd

	"""
        nav_roll        float   deg     Current desired roll
        nav_pitch       float   deg     Current desired pitch
        nav_bearing     int16_t deg     Current desired heading
        target_bearing  int16_t deg     Bearing to current waypoint/target
        wp_dist uint16_t        m       Distance to active waypoint
        alt_error       float   m       Current altitude error
        aspd_error      float   m/s     Current airspeed error
        xtrack_error    float   m       Current crosstrack error on x-y plane
        """
	reference = (msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.alt_error, msg.aspd_error, msg.xtrack_error)
        #print "\nRF_Roll\tRF_Pitch\tRF_Head\tRF_Alt\tRF_Spd\tRF_XY"
        #print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % reference

        #print("\nRoll error:%f, pitch error:%f, heading error:%f\n" %(abs(msg.nav_roll - current_roll), abs(msg.nav_pitch - current_pitch), abs(msg.nav_bearing - current_heading)))

	if avg_start == 1:
		ref_alt = 0.0
		ref_aspd = 0.0

		avg_refer_roll.append(msg.nav_roll)
		avg_refer_pitch.append(msg.nav_pitch)
		avg_refer_yaw.append(msg.nav_bearing)

		ref_aspd = current_aspd + msg.aspd_error
		avg_refer_aspd.append(ref_aspd)

		ref_alt = current_altitude + msg.alt_error
		avg_refer_alt.append(ref_alt)

#------------------------------------------------------------------------------------
def handle_position(msg):
        position_data = (msg.lat, msg.lon)

	#print(msg)
	#print("lat:%f, lon:%f" %(float(msg.lat), float(msg.lon)))
	
        global avg_start
        global position_cnt
        global avg_lat
        global avg_lon
	global avg_vertical_speed

	lat = 0
	lon = 0
	v_speed = 0
        if avg_start == 1:
                # Store average of hud data
                position_cnt += 1
		lat = msg.lat / 1000
		lat = lat * 1000
		lon = msg.lon / 1000
		lon = lon * 1000
		v_speed = msg.vz/100

                avg_lat.append(lat)
                avg_lon.append(lon)
		avg_vertical_speed.append(v_speed)

	#print("[Debig] Vertical Speed:%d" %v_speed)

#------------------------------------------------------------------------------------
def handle_status(msg):

	global Parachute
	global GPS_status
	global Accel_status
	global Gyro_status
	global PreArm_error
	global Baro_status

        status_data = (msg.severity, msg.text)
        print "[status_text] %s" % msg.text
	
	# Detecting a depolyed parachute
	if "Parachute: Released" in msg.text:
		Parachute = 1

	# Detecting an error on GPS
	elif "NavEKF" in msg.text and "lane switch" in msg.text:
		GPS_status = 0
	
	# Detecting an error on gyro sensor
	elif "Vibration compensation ON" in msg.text:
		Gyro_status = 0

	# Detecting an error on accelerometer sensor
	elif "EKF primary changed" in msg.text:
		Accel_status = 0

	# Detecting error messages related to a barometer sensor
	elif "PreArm: Waiting for Nav Checks" in msg.text:
		Baro_status = 0
	
	# Detecting a PreArm check error
	elif "PreArm: Check" in msg.text:
		PreArm_error = 1

#------------------------------------------------------------------------------------
def handle_time(msg):
	global system_time

	#time_data = (msg.time_unix_usec, msg.time_boot_ms)	
	system_time = msg.time_unix_usec
	#print "[system_time] UNIX time:%d, Boot time:%d" % time_data

#------------------------------------------------------------------------------------
def handle_mission(msg):
	global mission_cnt

	mission_cnt = msg.count
	print("[MISSION_COUNT]%d"%mission_cnt)

#------------------------------------------------------------------------------------
def handle_param(msg):

        global target_param
        global target_param_ready
        global target_param_value

        message = msg.to_dict()
        if message['param_id'].decode("utf-8") == target_param and target_param_ready == 0:
                target_param_ready = 1
                target_param_value = message['param_value']
        else:
                target_param_ready = 0

#------------------------------------------------------------------------------------
def handle_gps(msg):
	
	global avg_start
	global avg_num_GPS
	global avg_alt_GPS

	if avg_start == 1:
		avg_num_GPS.append(int(msg.satellites_visible))
		avg_alt_GPS.append(int(msg.alt/1000)) # convert mm unit to meter unit

	#print("# of GPS:%d, GPS altitude:%d"%(num_GPS, alt_GPS))
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
		elif msg_type == "GLOBAL_POSITION_INT":
			handle_position(msg)
		elif msg_type == "STATUSTEXT":
			handle_status(msg)
		elif msg_type == "SYSTEM_TIME":
			handle_time(msg)
		elif msg_type == "MISSION_COUNT":
			handle_mission(msg)
		elif msg_type == "PARAM_VALUE":
                        handle_param(msg)
		elif msg_type == "GPS_RAW_INT":
			handle_gps(msg)

#---------------------------- (End) READ STATES OF AP -----------------------------
	
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
def re_launch():

	global goal_throttle
	global current_flight_mode
	global drone_status

	print("#-----------------------------------------------------------------------------")
	print("#-----------------------------------------------------------------------------")
	print("#------------------------- RE-LAUNCH the vehicle -----------------------------")
	print("#-----------------------------------------------------------------------------")
	print("#-----------------------------------------------------------------------------")
	
	
	# Step 1. Land the vehicle
	master.mav.set_mode_send(
	    master.target_system,
	    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
		
	# Wait for finishing the landing
	while True:
		if current_flight_mode == "LAND" and drone_status == 3:
			print("### Vehicle is grounded ###")
			break
		time.sleep(0.2)

	time.sleep(5)
	
	# Refer to https://discuss.ardupilot.org/t/arming-problem-fs-thr-value/806/2
	print("[re-launch] min_thr:%d, target throttle:%d" % (required_min_thr, 995))
	set_rc_channel_pwm(1, 1500)
	set_rc_channel_pwm(2, 1500)
	set_rc_channel_pwm(4, 1500)
        
	time.sleep(3)
	
        goal_throttle = 1000
	# Step 2. Reboot the RV's control program
	master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                             1, 0, 0, 0, 0, 0, 0)
	time.sleep(40)
	
	# Step 4. Re-take off the vehicle
	master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)

        # Wait for finishing the landing
        while True:
                if current_flight_mode == "GUIDED":
                        break
                time.sleep(0.2)
	
        # Reset preconditions to fuzz the target policy
        global Precondition_path
        set_preconditions(Precondition_path)

	# Arming
	master.mav.command_long_send(
    		master.target_system,
		master.target_component,
	    	mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    		0,
    		1, 0, 0, 0, 0, 0, 0)

	
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
                100) # param7- altitude

	time.sleep(5)
	goal_throttle = 1500

def init_files():

	path = ""
        path += "./results/"

	file_path = path + "roll.txt"
	f = open(file_path,"w")
	f.close()

        file_path = path + "pitch.txt"
        f = open(file_path,"w")
        f.close()
	
	file_path = path + "throttle.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "yaw.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "speed.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "altitude.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "position.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "status.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "gyro.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "accel.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "baro.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "GPS.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "parachute.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "pre_arm.txt"
        f = open(file_path,"w")
        f.close()

	file_path = path + "mission.txt"
        f = open(file_path,"w")
        f.close()

#------------------------------------------------------------------------------------
def store_result(state, Input):

	file_path = ""
	file_path += "./results/"

	# states related to roll angles
	if state == 0 or state == 12 or state == 15 or state == 18:
		file_path += "roll.txt"

	elif state == 1 or state == 13 or state == 16 or state == 19:
		file_path += "pitch.txt"

	elif state == 2 or state == 7:
		file_path += "throttle.txt"
	
	elif state == 3 or state == 6 or state == 14 or state == 17 or state == 20:
		file_path += "yaw.txt"	
	
	elif state == 4 or state == 5 or state == 22:
		file_path += "speed.txt"

	elif state == 8 or state == 11 or state == 21 or state == 32 or state == 33:
		file_path += "altitude.txt"

	elif state == 9 or state == 10:
		file_path += "position.txt"

	elif state == 23:
		file_path += "status.txt"

	elif state == 24:
		file_path += "gyro.txt"

	elif state == 25:
		file_path += "accel.txt"

	elif state == 26:
		file_path += "baro.txt"

	elif state == 27 or state == 31:
		file_path += "GPS.txt"
	
	elif state == 28:
		file_path += "parachute.txt"
	
	elif state == 29:
		file_path += "pre_arm.txt"

	elif state == 30:
		file_path += "mission.txt"

	# Prevent logging redundant inputs on each state file
	log_flag = 1

	read_f = open(file_path,"r")
	for line in read_f.readlines():
        	if Input in line:	
 			log_flag = 0
	read_f.close()

	if log_flag == 1:
		f = open(file_path,"a")
		f.write(Input)
		f.write("\n")
		f.close()



#------------------------------------------------------------------------------------
def profile(input_type, targetInput, input_value):

	global avg_start
	global avg_airspeed
	global avg_groundspeed
	global avg_heading
	global avg_throttle
	global avg_alt
	global avg_climb
	global hud_cnt

	global avg_roll
	global avg_pitch
	global avg_yaw
	global avg_rollspeed
	global avg_pitchspeed
	global avg_yawspeed
	global atti_cnt

	global position_cnt
	global avg_lat
	global avg_lon

	global avg_rc1
	global avg_rc2
	global avg_rc3
	global avg_rc4

	global avg_refer_roll
	global avg_refer_pitch
	global avg_refer_yaw
	global avg_refer_aspd
	global avg_refer_alt
	
	global avg_num_GPS
	global avg_alt_GPS
	global avg_vertical_speed

	global Parachute
	global GPS_status
	global Accel_status
	global Gyro_status
	global Baro_status
	global PreArm_error

	global Measuring_iteration
	global goal_throttle
	global SD_baseline
	global SD_target_input
	global start_profiling

	for iteration in range(start_profiling, 2):
		count = 0
		itera = 0
		# range(x): from 0 to (x-1)
		if iteration == 0:
			itera = Measuring_iteration + 2
		else:
			itera = Measuring_iteration

		for i in range(itera):
			if iteration == 0:
				print("------------------(Build baseline states)------------------")
			else:
				print("------------------(Measuring states while executing an input)------------------")

			print("%d iteration!"%(i+1))
			print("[Debug] start_profiling: %d"%start_profiling)

			# from 0 to 20
			for mode_i in range(0, 21):

				# 8, 10, 12 are empty mode
				if mode_i == 8 or mode_i == 10 or mode_i == 12 or mode_i == 3 or mode_i == 14 or mode_i == 15 or mode_i == 18:	
					continue

				master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_i)
				time.sleep(0.2)
			
				# Execute a user command or change an environmental factor		
				# When 'iteration' is 0, we build a baseline
				if iteration != 0 and input_type == "env":
					rand_value = 0
					rand_value = random.randint(0, 100)
		        		print("[Test] %s: %d" % (targetInput, rand_value))
				        
					master.mav.param_set_send(master.target_system, 
					master.target_component, targetInput, 
					rand_value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

				if iteration != 0 and input_type == "cmd":
					if targetInput == "RC1" or targetInput == "RC2" or targetInput == "RC4":
						target_RC = 0
						mutated_value = 0
						mutated_value = random.randint(1000, 2000)
						
						if targetInput == "RC1":
							target_RC = 1
						elif targetInput == "RC2":
							target_RC = 2
						elif targetInput == "RC4":
							target_RC = 4
					
						set_rc_channel_pwm(target_RC, mutated_value)
						print("[Test] Change RC%d = %d"%(target_RC, mutated_value))

				if targetInput == "RC3" and input_type == "cmd" and iteration != 0:
					goal_throttle = random.randint(1500, 2000)
                                        print("[Test] Throttle:%d" %goal_throttle)

				elif targetInput == "MAV_CMD_DO_PARACHUTE" and input_type == "cmd" and iteration != 0:
					master.mav.command_long_send(
                			master.target_system,  # target_system
			                master.target_component, # target_component
			                mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
			                0, 2, 0, 0, 0, 0, 0, 0)

				elif input_type == "cmd" and input_value >= 1 and iteration != 0:
					rand = []
					for i in range(7):
		                                rand.append(random.randint(1, 100))
					
					master.mav.command_long_send(
			                master.target_system,  # target_system
			                master.target_component, # target_component
			                input_value,
			                0,
			                rand[0], rand[1], rand[2], rand[3], rand[4], rand[5], rand[6])
					print("[Test] %s (%d, %d, %d, %d, %d, %d, %d)"%(targetInput, rand[0], rand[1], rand[2], rand[3], rand[4], rand[5], rand[6]) )

				count += 1
				avg_start = 1
				time.sleep(Measuring_duration)
				avg_start = 0

				sd[0] = Standard_deviation(avg_rc1) #0
			        sd[1] = Standard_deviation(avg_rc2) #1
			        sd[2] = Standard_deviation(avg_rc3) #2
			        sd[3] = Standard_deviation(avg_rc4) #3
				sd[4] = Standard_deviation(avg_airspeed) #4
				sd[5] = Standard_deviation(avg_groundspeed) #5
				sd[6] = Standard_deviation(avg_heading) #6
				sd[7] = Standard_deviation(avg_throttle) #7
				sd[8] = Standard_deviation(avg_alt) #8
				sd[9] = Standard_deviation(avg_lat) #9
				sd[10] = Standard_deviation(avg_lon) #10
				sd[11] = Standard_deviation(avg_climb) #11
				sd[12] = Standard_deviation(avg_roll) #12
				sd[13] = Standard_deviation(avg_pitch) #13
				sd[14] = Standard_deviation(avg_refer_yaw) #14
				sd[15] = Standard_deviation(avg_rollspeed) #15
				sd[16] = Standard_deviation(avg_pitchspeed) #16
				sd[17] = Standard_deviation(avg_yawspeed) #17
				sd[18] = Standard_deviation(avg_refer_roll) #18
				sd[19] = Standard_deviation(avg_refer_pitch) #19
				sd[20] = Standard_deviation(avg_refer_yaw) #20
				sd[21] = Standard_deviation(avg_refer_alt) #21
				sd[22] = Standard_deviation(avg_refer_aspd) #22
				sd[23] = drone_status #23
				sd[24] = Gyro_status #24
				sd[25] = Accel_status #25
				sd[26] = Baro_status #26
				sd[27] = GPS_status #27
				sd[28] = Parachute #28
				sd[29] = PreArm_error #29
				sd[30] = mission_cnt #30
				sd[31] = Standard_deviation(avg_num_GPS) #31
				sd[32] = Standard_deviation(avg_alt_GPS) #32
				sd[33] = Standard_deviation(avg_vertical_speed) #33

				if iteration == 0:
					# from 0 to 33
				      	for index in range(number_of_states):
						SD_baseline[index] += sd[index]  
						#print("[%d] %f"%(index, SD_baseline[index]))
				else:
					# from 0 to 30
		                        for index in range(number_of_states):
		                                SD_target_input[index] += sd[index]

			

				hud_cnt = 0
				atti_cnt = 0
				position_cnt = 0
				avg_airspeed = []
				avg_groundspeed = []
				avg_heading = []
				avg_throttle = []
				avg_alt = []
				avg_climb = []

				avg_roll = []
				avg_pitch = []
				avg_yaw = []
				avg_rollspeed = []
				avg_pitchspeed = []
				avg_yawspeed = []
	
				avg_lat = []
				avg_lon = []

				avg_rc1 = []
				avg_rc2 = []
				avg_rc3 = []
				avg_rc4 = []
	
				avg_refer_roll = []
				avg_refer_pitch = []
				avg_refer_yaw = []
				avg_refer_aspd = []
				avg_refer_alt = []
		
				avg_num_GPS = []
				avg_alt_GPS = []
				avg_vertical_speed = []

				Parachute = 0
				GPS_status = 1
				Accel_status = 1
				Gyro_status = 1
				Baro_status = 1
				PreArm_error = 0	
				print ("--------------- [End] measuring states in %s mode (%d)---------------"%(current_flight_mode, mode_i))



		print "---------------------------------------------"
		print("count:%d" %count)	
		if iteration == 0:
			print("############### Baseline ###############")
			for l in range(number_of_states):
				SD_baseline[l] = SD_baseline[l] / count 
				sys.stdout.write("(%d)%f " %(l, SD_baseline[l]))

			re_launch()
		else:
			print("############### target input:%s ###############"%targetInput)
			for l in range(number_of_states):
		                SD_target_input[l] = SD_target_input[l] / count 
		                sys.stdout.write("(%d)%f " %(l, SD_target_input[l]))

		print "---------------------------------------------"
		sys.stdout.flush()

		global minimum_sd 
		for l in range(number_of_states):
			if SD_target_input[l] > minimum_sd and (abs(SD_baseline[l] - SD_target_input[l])) > SD_baseline[l]:
				store_result(state=l, Input = targetInput)
				print("Changed state:%d (SD: %f)"% (l, SD_target_input[l]))
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
                100) # param7- altitude
"""
ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=False)
    ack_msg = ack_msg.to_dict()

    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

"""
time.sleep(5)

# Maintain mid-position of stick on RC controller 
goal_throttle = 1500
t1 = threading.Thread(target=throttle_th, args=())
t1.daemon = True 
t1.start()

mode_id = master.mode_mapping()['STABILIZE']
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

parsing_command("cmds.txt")
print(env_name)
parsing_env("envs.txt")
print(cmd_name)
print(cmd_number)

# Create files for logging results
init_files()

sd = []
for i in range(number_of_states):
	sd.append(0)
	SD_baseline.append(0)
	SD_target_input.append(0)

set_preconditions(Precondition_path)

original_val = 0.0
"""
for i in range(len(cmd_name)):

        print("[Analyzing user commands] %s" %cmd_name[i])
        profile(input_type="cmd", targetInput=cmd_name[i], input_value=cmd_number[i])
        start_profiling = 1

        re_launch()
"""

#--------------------------------------------------------
for i in range(len(env_name)):
	target_param = env_name[i]
        cnt = 0
        while target_param_ready == 0 and cnt < 10:
                time.sleep(1)
                cnt += 1

	original_val = target_param_value

	profile(input_type ="env", targetInput=env_name[i], input_value=0)
	
	# Restore original states
	master.mav.param_set_send(master.target_system,
                                  master.target_component, target_param,
                                  original_val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

	print("[Debug] %s = %f" %(target_param, original_val))
	time.sleep(3)	
	start_profiling = 1

	re_launch()

	target_param_ready = 0
	target_param_value = 0


#--------------------------------------------------------

for i in range(len(cmd_name)):

        print("[Analyzing user commands] %s" %cmd_name[i])
        profile(input_type="cmd", targetInput=cmd_name[i], input_value=cmd_number[i])
        start_profiling = 1

        re_launch()

#------------------------------------------------------------------------------------------------		
print("##################### the end of dynamic analysis #####################")
while True:
	time.sleep(1)
