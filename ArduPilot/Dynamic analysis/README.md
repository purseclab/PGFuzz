# Purpose of dynamic analysis
This dynamic analysis for mapping user commands / environmental factors to each state (term).
This version support total 34 states as in the below.

(State number), (state name)<br>
0,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;RC1<br>
1,&emsp;RC2<br>
2,&emsp;RC3<br>
3,&emsp;RC4<br>
4,&emsp;Air speed<br>
5,&emsp;Ground speed<br>
6,&emsp;Heading<br>
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

# Execution results
This program classifies same type of states as one type of the state.
For example, we consider 'roll', 'reference roll', and 'roll speed' as 'roll' state.
The results of the analysis will be store in ./results/*.txt
Each txt file contains user commands and/or environmental factors which affect the state.
For instance, 'roll.txt' includes all inputs which can change the roll state.
This version categorizes inputs to 15 states as in the below.

roll.txt<br>
pitch.txt<br>
throttle.txt<br>
yaw.txt<br>
speed.txt<br>
altitude.txt<br>
position.txt<br>
status.txt<br>
gyro.txt<br>
accel.txt<br>
baro.txt<br>
GPS.txt<br>
parachute.txt<br>
pre_arm.txt<br>
mission.txt<br>

# Setup (ArduPilot)
This program is based on Python 2.7.

## Setting up a simulator
- <a href="https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html" target="_blank"> Setting up SITL on Linux </a>
- <a href="https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html" target="_blank"> Tutorial</a>

## Setting up preconditions to test user commands
preconditions.txt file contain some configuration parameters to set sensors or devices. Some user commands require such a configuration. For example, the current version of the file includes parameters to set a parachute. 
```bash
cat ./preconditions.txt
CHUTE_ENABLED 1
CHUTE_TYPE 10
SERVO9_FUNCTION 27
SIM_PARA_ENABLE 1
SIM_PARA_PIN 9
```
If you want to add additional devices (e.g., range beacons or optical flow sensors), please refer to the following <a href="https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html" target="_blank">documentation</a> and add configuration parameters to the preconditions.txt file.

## How to execute it?
### First terminal window
```bash
~/ardupilot/Tools/autotest$ ./sim_vehicle.py -v ArduCopter --console -w --map
```

### Second terminal window <br>
Please note that before you execute dynamic analysis, you need to wait for finishing to setup SITL simulation loading. After you see "EKF2 IMU1 is using GPS" messages on SITL, you can execute dynamic analysis.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/Dynamic%20analysis/example/dynamic_ex1.jpg"> <br>


```bash
cd ~/PGFUZZ/ArduPilot/Dynamic analysis/
python profiling_cmd_env.py
```

<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/Dynamic%20analysis/example/dynamic_ex2.jpg">

After the dynamic analysis finishes to map a input to states, it will show the mapping results as follows.<br>
For example, the changed RC 3 (throttle) value affects RC1, RC2, RC3, throttle, altitude, latitude, climb rate, reference altitude, GPS altitude, and vertical speed states.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/Dynamic%20analysis/example/dynamic_ex3.jpg"><br>

## Configurations
### When you want to increase accuracy of this dynamic analysis
You can increase the number of iterations and measuring duration for each user command and environmental factor.
```bash
vim +114 profiling_cmd_env.py 
```
You can modify 'Measuring_duration' and/or 'Measuring_iteration'.
