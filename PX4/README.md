# PGFUZZ for PX4

## Parsing valid ranges of configuration parameters
Please refer to <a href="https://github.com/purseclab/PGFuzz/tree/main/PX4/xml_parse" target="_blank"> this</a>.

## Mapping inputs to states (terms)
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/Dynamic%20analysis" target="_blank"> this</a>.

## How to execute it?
### Download PX4
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git px4_pgfuzz
cd px4_pgfuzz
git checkout 9524e8ec032c9c4c9cd8b8860a1675ab998e9997
git submodule update --init --recursive
```

### Check whether PX4 works well on your environment
```bash
cd ~/px4_pgfuzz/
make px4_sitl_default jmavsim 
```
### Add environment variables
You must point to your pgfuzz and PX4 directories.
```bash
export PGFUZZ_HOME=/home/pgfuzz/pgfuzz/
export PX4_HOME=/home/pgfuzz/px4_pgfuzz/
```

### Execute PGFUZZ
```bash
cd ~/pgfuzz/px4_pgfuzz/
python2 pgfuzz.py -i false
```
Here, 'i' option decides whether the input mutation will be bounded or unbounded.

It creates two terminal windows.

### First terminal window
It is a SITL Simulator (Software in the Loop). You can monitor the RV's current states such as altitude and attitude.<br>

### Second terminal window
It shows status of fuzzing such as mutated inputs, violated policies, propositional, and global distances.<br>
