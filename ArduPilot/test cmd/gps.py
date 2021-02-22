# Import mavutil

from pymavlink import mavutil
import time

# Global variables
timestamp = 1590631484
GPStime = 353268
alt = 584.0
# Create the connection
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# GPS_TYPE need to be MAV
while True:
	time.sleep(0.2)
	timestamp -= 1
	GPStime -= 1
	alt -= 0.5
	master.mav.gps_input_send(
        timestamp,          #Timestamp (micros since boot or Unix epoch)
        0,          #ID of the GPS for multiple GPS inputs
        8|16|32,    #Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
        GPStime,          #GPS time (milliseconds from start of GPS week)
        2,          #GPS week number
        3,          #0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        0,          #Latitude (WGS84), in degrees * 1E7
        0,          #Longitude (WGS84), in degrees * 1E7
        alt,          #Altitude (AMSL, not WGS84), in m (positive for up)
        1,          #GPS HDOP horizontal dilution of position in m
        1,          #GPS VDOP vertical dilution of position in m
        0,          #GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,          #GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,          #GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,          #GPS speed accuracy in m/s
        0,          #GPS horizontal accuracy in m
        0,          #GPS vertical accuracy in m
        7           #Number of satellites visible.
    )
