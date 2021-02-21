This dynamic analysis for mapping user commands / environmental factors to each state (term).
This version support total 34 states as in the below.
-------------------
RC1
RC2
RC3
RC4
Air speed
Ground speed
Heading	
Throttle
Altitude
Latitude
Longitude
Climb rate
Roll
Pitch
Yaw
Roll speed
Pitch speed
Yaw speed
Reference roll
Reference pitch
Reference Yaw
Reference altitude
Reference air speed
Status
Gyroscope/accelerometer
accelerometer/magnetometer
Barometer
GPS
Parachte
Pre-arm checking
mission
num_GPS
ALT_GPS
Z_speed
-------------------
This program classifies same type of states as one type of the state.
For example, we consider 'roll', 'reference roll', and 'roll speed' as 'roll' state.
The results of the analysis will be store in ./results/*.txt
Each txt file contains user commands and/or environmental factors which affect the state.
For instance, 'roll.txt' includes all inputs which can change the roll state.
This version categorizes inputs to 15 states as in the below.
-------------------
roll.txt
pitch.txt
throttle.txt
yaw.txt
speed.txt
altitude.txt
position.txt
status.txt
gyro.txt
accel.txt
baro.txt
GPS.txt
parachute.txt
pre_arm.txt
mission.txt
-------------------
