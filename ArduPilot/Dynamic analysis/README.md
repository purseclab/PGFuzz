# Purpose of dynamic analysis
This dynamic analysis is for mapping user commands / environmental factors to each state (term).
This version support total 34 states as in the below.

(State number), (state name)<br>
0,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;RC1<br>
1,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;RC2<br>
2,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;RC3<br>
3,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;RC4<br>
4,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Air speed<br>
5,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Ground speed<br>
6,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Heading<br>
7,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Throttle<br>
8,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Altitude<br>
9,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Latitude<br>
10,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Longitude<br>
11,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Climb rate<br>
12,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Roll<br>
13,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Pitch<br>
14,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Yaw<br>
15,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Roll speed<br>
16,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Pitch speed<br>
17,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Yaw speed<br>
18,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Reference roll<br>
19,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Reference pitch<br>
20,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Reference Yaw<br>
21,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Reference altitude<br>
22,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Reference air speed<br>
23,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Status<br>
24,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Gyroscope/accelerometer<br>
25,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;accelerometer/magnetometer<br>
26,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Barometer<br>
27,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;GPS<br>
28,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Parachte<br>
29,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Pre-arm checking<br>
30,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;mission<br>
31,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;num_GPS<br>
32,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;ALT_GPS<br>
33,&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Vertical speed<br>

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
~/ardupilot/Tools/autotest$ ./sim_vehicle.py -v ArduCopter --console -w --map --speedup=2
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
For example, the changed RC 1 (roll angle) value affects RC1, air speed, ground speed, heading, throttle, altitude, latitude, longitude, climb rate, roll, yaw, roll speed, yaw speed, reference roll, reference yaw, reference altitude, reference air speed, altitude measured from GPS, and vertical speed states
.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/Dynamic%20analysis/example/dynamic_ex3.jpg"><br>

## Configurations
### When you want to increase accuracy of this dynamic analysis
You can increase the number of iterations and measuring duration for each user command and environmental factor.
```bash
vim +114 profiling_cmd_env.py 
```
You can modify 'Measuring_duration' and/or 'Measuring_iteration'.

### If you want to add or remove inputs
You can add or remove a user command (or environmeantal factor) from cmds.txt / envs.txt.
