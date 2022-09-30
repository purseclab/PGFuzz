#!/usr/bin/env python

from pymavlink import mavutil
from pymavlink import mavwp
import time
import threading
import string
import sys

# Global variables
target_roll = 1500
target_pitch = 1500
target_yaw = 1500
target_throttle = 1600
target_rc5 = 1500
#------------------------------------------------------------------------------------
def throttle_th():
    global target_roll
    global target_pitch
    global target_yaw
    global target_throttle

    while True:
        master.mav.rc_channels_override_send(
                master.target_system, 
                master.target_component, 
                target_roll, target_pitch, target_throttle, target_yaw, target_rc5, 1500, 1500, 1500)
        time.sleep(0.2)

#------------------------------------------------------------------------------------
print(sys.version)

# python /usr/local/bin/mavproxy.py --mav=/dev/tty.usbserial-DN01WM7R --baudrate 57600 --out udp:127.0.0.1:14540 --out udp:127.0.0.1:14550
connection_string = '0.0.0.0:14540'
master = mavutil.mavlink_connection('udp:'+connection_string)

master.wait_heartbeat()
print("HEARTBEAT OK\n")

goal_throttle = 1500


#while True:
#    master.mav.manual_control_send(
#        master.target_system,
#        0,  # pitch
#        0,  # roll
#        0,  # throttle
#        0,  # twisting of the joystick
#        0)    # 1 for pressed, 0 for released
t = threading.Thread(target=throttle_th, args=())
t.daemon = True 
t.start()


while True:
    target = raw_input("[pitch/roll/throttle/yaw/mode] ")
    value = raw_input("[value] ")
    value = int(value)
    print('%s = %d' %(target, value));

    if target == "pitch":
        target_pitch = value
    elif target == "roll":
        target_roll = value
    elif target == "throttle":
        target_throttle = value
    elif target == "yaw":
        target_yaw = value
    elif target == "mode":
        target_rc5 = value

