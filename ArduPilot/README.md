# PGFUZZ for ArduPilot
## How to execute it?
```bash
cd ~/PGFUZZ/ArduPilot/
python python pgfuzz.py
```

It creates two terminal windows.

### First terminal window
It is a SITL Simulator (Software in the Loop). You can monitor the RV's current states such as altitude and attitude.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex1.jpg">

### Second terminal window
It shows status of fuzzing such as mutated inputs, violated policies, propositional, and global distances.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex2.jpg">
