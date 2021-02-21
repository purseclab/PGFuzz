## Purpose of dynamic analysis
This dynamic analysis for mapping user commands / environmental factors to each state (term).
This version support total 34 states as in the below.

RC1<br>
RC2<br>
RC3<br>
RC4<br>
Air speed<br>
Ground speed<br>
Heading<br>
Throttle<br>
Altitude<br>
Latitude<br>
Longitude<br>
Climb rate<br>
Roll<br>
Pitch<br>
Yaw<br>
Roll speed<br>
Pitch speed<br>
Yaw speed<br>
Reference roll<br>
Reference pitch<br>
Reference Yaw<br>
Reference altitude<br>
Reference air speed<br>
Status<br>
Gyroscope/accelerometer<br>
accelerometer/magnetometer<br>
Barometer<br>
GPS<br>
Parachte<br>
Pre-arm checking<br>
mission<br>
num_GPS<br>
ALT_GPS<br>
Z_speed<br>

## Execution results
This program classifies same type of states as one type of the state.
For example, we consider 'roll', 'reference roll', and 'roll speed' as 'roll' state.
The results of the analysis will be store in ./results/*.txt
Each txt file contains user commands and/or environmental factors which affect the state.
For instance, 'roll.txt' includes all inputs which can change the roll state.
This version categorizes inputs to 15 states as in the below.

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
