# PGFuzz for ArduPilot

## Parsing valid ranges of configuration parameters
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/xml_parse" target="_blank"> this</a>.

## Mapping inputs to states (terms)
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/Dynamic%20analysis" target="_blank"> this</a>.

## How to execute it?
### Download ArduPilot
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git ardupilot_pgfuzz
cd ardupilot_pgfuzz
git checkout ea559a56aa2ce9ede932e22e5ea28eb1df07781c
git submodule update --init --recursive
```

### Check whether ArduPilot works well on your environment
```bash
cd ~/ardupilot_pgfuzz/
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w
```

### Add environment variables
You must point to your pgfuzz and ArduPilot directories.
```bash
export PGFUZZ_HOME=/home/pgfuzz/pgfuzz/
export ARDUPILOT_HOME=/home/pgfuzz/ardupilot_pgfuzz/
```

### Execute PGFuzz
```bash
cd ~/pgfuzz/ArduPilot/
python2 pgfuzz.py
```

It creates two terminal windows.

### First terminal window
It is a SITL Simulator (Software in the Loop). You can monitor the RV's current states such as altitude and attitude.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex1.jpg">

### Second terminal window
It shows status of fuzzing such as mutated inputs, violated policies, propositional, and global distances.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex2.jpg">


## Evaluate the noise elimination component
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/EEN" target="_blank"> this</a>.

