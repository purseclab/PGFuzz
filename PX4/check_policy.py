#!/usr/bin/env python

import sys, os
from optparse import OptionParser

# tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil
import time
import sys

#------------------------------------------------------------------------------------
# Global variables
current_altitude = 0
previous_altitude = 0
current_flight_mode = ""
RTL_alt = 10
#------------------------------------------------------------------------------------
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
	channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, 
			msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)
	time.sleep(0.01)

#------------------------------------------------------------------------------------
def handle_hud(msg):
	hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
				msg.throttle, msg.alt, msg.climb)
	print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
	print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

	global current_altitude 
	global previous_altitude
	
	previous_altitude = current_altitude
	current_altitude = msg.alt
	time.sleep(0.01)

#------------------------------------------------------------------------------------
def handle_attitude(msg):
	attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
				msg.pitchspeed, msg.yawspeed)
	#print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
	#print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

#------------------------------------------------------------------------------------
def check_policy():
	global current_altitude
	global previous_altitude
	# 1) Flip mode policy
	# 594: 10 meters
        if current_altitude < 594 and current_flight_mode == "FLIP": 
                print("Flip policy is violated! Flip mode was triggered when altitude was %f" % current_altitude)
		sys.exit()
	elif current_flight_mode == "RTL" and current_altitude >= RTL_alt:
		if previous_altitude != current_altitude:
			print("RTL policy is violated! ArduPilot maintains the current altitude, if the current altitude (%f) is higher than RTL_ALT (%f)" %(current_altitude, RTL_alt))
			sys.exit()
	#else:
		#print("Altitude: %f, Mode: %s" %(current_altitude, current_flight_mode))
	time.sleep(0.01)

#------------------------------------------------------------------------------------
def read_loop(m):
	while(True):

		# grab a mavlink message
		msg = m.recv_match(blocking=True)

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
		
		# Check whether current status has a policy violation or not
		check_policy()

#------------------------------------------------------------------------------------

connection_string = '0.0.0.0:14540'
master = mavutil.mavlink_connection('udp:'+connection_string)

# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("HEARTBEAT OK\n")

# request data to be sent at the given rate
for i in range(0, 3):
    master.mav.request_data_stream_send(master.target_system, master.target_component, 
		mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)

# Request parameter
master.mav.param_request_read_send(master.target_system, master.target_component, 'RTL_ALT',-1)
#message = master.recv_match(type='PARAM_VALUE', blocking=True)
#message = message.to_dict()

#RTL_alt = message['param_value']
#RTL_alt = (RTL_alt/100)+584

#print('(Before) name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), RTL_alt))

print("read loop")
# enter the data loop
read_loop(master)

