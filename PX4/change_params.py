import sys
from pymavlink import mavutil
from pymavlink import mavwp
import time
import threading
#------------------------------------------------------------------------------------
# Global variables
base_mode = [157, 193, 209, 217]
flight_mode = ["manual", "stabilized", "acro", "rattitude", "altitude", "offboard", "position", "hold", "missition", "return", "follow_me"]
current_altitude = 0
previous_altitude = 0
current_flight_mode = ""
RTL_alt = 10

paramsName = []
Min_param = -999999999999999999
Max_param = 999999999999999999
target_roll = 1500
target_pitch = 1500
target_yaw = 1500
target_throttle = 1800


#------------------------------------------------------------------------------------    
# python /usr/local/bin/mavproxy.py --mav=/dev/tty.usbserial-DN01WM7R --baudrate 57600 --out udp:127.0.0.1:14540 --out udp:127.0.0.1:14550
#connection_string = '0.0.0.0:14540'
connection_string = '0.0.0.0:14550'
master = mavutil.mavlink_connection('udp:'+ connection_string)

master.wait_heartbeat()
print("HEARTBEAT OK\n")


while True:
        master.mav.param_set_send(master.target_system, master.target_component,
                                'MC_PITCHRATE_FF',
                                9999,
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                                
        time.sleep(0.5)

