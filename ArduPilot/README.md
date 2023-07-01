# PGFuzz for ArduPilot

## 1. Parsing valid ranges of configuration parameters
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/xml_parse" target="_blank"> this</a>.

## 2. Mapping inputs to states (terms)
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/Dynamic%20analysis" target="_blank"> this</a>.

## 3. How to execute it?
### 3-1) Download ArduPilot
**Option1**: If you want to test a specific software version, please checkout a commit hash after cloning ArduPilot.
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git ardupilot_pgfuzz
cd ardupilot_pgfuzz
git checkout ea559a56aa2ce9ede932e22e5ea28eb1df07781c
git submodule update --init --recursive
```

**Option2**: If you want to test the latest version, please update submodule without the checkout step.
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git ardupilot_pgfuzz
cd ardupilot_pgfuzz
git submodule update --init --recursive
```

### 3-2) Check whether ArduPilot works well on your environment
```bash
cd ~/ardupilot_pgfuzz/
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w
```

### 3-3) Add environment variables
You must point to your pgfuzz and ArduPilot directories.
```bash
export PGFUZZ_HOME=/home/[Your user account name]/PGFuzz/PGFuzz/
export ARDUPILOT_HOME=/home/[Your user account name]/PGFuzz/ardupilot_pgfuzz/
```
e.g., my user account name is 'hskim'.
```bash
export PGFUZZ_HOME=/home/hskim/PGFuzz/PGFuzz/
export ARDUPILOT_HOME=/home/hskim/PGFuzz/ardupilot_pgfuzz/
```

### 3-4) Execute PGFuzz
```bash
cd ~/PGFuzz/ArduPilot/
python2 pgfuzz.py
```

It creates two terminal windows.

### First terminal window
It is a SITL Simulator (Software in the Loop). You can monitor the RV's current states such as altitude and attitude.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex1.jpg">

### Second terminal window
It shows status of fuzzing such as mutated inputs, violated policies, propositional, and global distances.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex2.jpg">

### Caution
Please do not turn on "Virtual Joystick" on QGroundControl while executing PGFuzz. The drone will fail to takeoff from the ground.

## 4. Evaluate the noise elimination component
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/EEN" target="_blank"> this</a>.

## 5. How to add a new policy?
5-1) Add a predicate to 'calculate_distance' function in 'fuzzing.py'

5-2) Change 'Current_policy' and 'Current_policy_P_length' values in 'fuzzing.py' according to the new policy
