## Purpose of the noise elimination
This is for eliminating the environmental noise such as sensor noise and wind effect. <br>
Without noise elimination, PGFUZZ may incorrectly guide the mutation engine.

## How to execute it?
### First terminal window
```bash
~/ardupilot/Tools/autotest$ ./sim_vehicle.py -v ArduCopter --console -w --map
```

### Second terminal window <br>
Please note that before you execute dynamic analysis, you need to wait for finishing to setup SITL simulation loading. <br>
After you see "EKF2 IMU1 is using GPS" messages on SITL, you can execute the noise elimination.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/Dynamic%20analysis/example/dynamic_ex1.jpg"> <br>


```bash
cd ~/PGFUZZ/ArduPilot/EEN/
python EEN.py
```
