#!/usr/bin/python

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
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# states - 0: turn off, 1: turn on
Parachute_on = 0
Armed = 0
current_flight_mode = ""
previous_altitude = 0
current_altitude = 0
drone_status = 0
current_rc_3 = 0

current_roll = 0.0
current_pitch = 0.0
current_heading = 0.0

alt_error = 0.0
roll_error = 0.0
pitch_error = 0.0
heading_error = 0.0
stable_counter = 0

# Distance
Propositional_distance = []
Global_distance = 0

target_param = ""
target_param_ready = 0
target_param_value = 0

def calculate_distances():

	print("[Debug] calculate_distances() is called.")

        # A.CHUTE1 policy
        global Parachute_on
        global Armed
        global current_flight_mode
        global previous_altitude
        global current_altitude
        global Propositional_distance
        global Global_distance
	global target_param

        # Propositional distances
        # 0: turn off, 1: turn on

        # P1
        if Parachute_on == 1:
                P1 = 1
        else:
                P1 = -1
        # P2
        if Armed == 0:
                P2 = 1
        else:
                P2 = -1
        # P3
        if current_flight_mode == "FLIP" or current_flight_mode == "ACRO":
                P3 = 1
        else:
                P3 = -1

        # P4
        P4 = (current_altitude - previous_altitude) / current_altitude

        # P5
        # Request parameter
        master.mav.param_request_read_send(master.target_system, master.target_component, 'CHUTE_ALT_MIN',-1)

        target_param = "CHUTE_ALT_MIN"
        count = 0
        while target_param_ready == 0 and count < 5:
                time.sleep(1)
                count += 1


        P5 = (target_param_value - current_altitude) / target_param_value

        Global_distance = -1 * (min(P1, max(P2, P3, P4, P5)))

        # Print distances
        print("#-----------------------------------------------------------------------------")
        print('[Distance] P1:%f, P2:%f, P3:%f, P4:%f, P5:%f' %(P1, P2, P3, P4, P5))
        print('[Distance] Global distance: %f' %Global_distance)
        print("#-----------------------------------------------------------------------------")

        if Global_distance < 0:
                print("***************Policy violation!***************")
                print("***************Policy violation!***************")
                print("***************Policy violation!***************")

        target_param_ready = 0
        target_param =""
        target_param_value = 0

#------------------------------------------------------------------------------------
#--------------------- (Start) READ Robotic Vehicle's states ------------------------
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

        hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                                msg.throttle, msg.alt, msg.climb)
        #print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
        #print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

        print("Alt: %f" %msg.alt)

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

        global current_roll
        global current_pitch
        global alt_error
        global roll_error
        global pitch_error
        global heading_error
        global stable_counter

        #print("altitude error:%f" %msg.alt_error)
        #msg.alt_error
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

        alt_error += msg.alt_error
        roll_error += abs(msg.nav_roll - current_roll)
        pitch_error += abs(msg.nav_pitch - current_pitch)
        heading_error += abs(msg.nav_bearing - current_heading)
        stable_counter += 1

        #print("Desired heading:%f, current heading:%f" %(msg.nav_bearing, current_heading))

#------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------
def handle_param(msg):

        global target_param
        global target_param_ready
        global target_param_value

        message = msg.to_dict()
        if message['param_id'].decode("utf-8") == target_param:
                target_param_ready = 1
                target_param_value = message['param_value']
        else:
                target_param_ready = 0

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
                elif msg_type == "PARAM_VALUE":
                        handle_param(msg)

#--------------------- (End) READ Robotic Vehicle's states ------------------------

def start_monitoring():
	master.wait_heartbeat()

	# request data to be sent at the given rate
	for i in range(0, 3):
        	master.mav.request_data_stream_send(master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)

	t2 = threading.Thread(target=read_loop, args=())
	t2.daemon = True
	t2.start()
