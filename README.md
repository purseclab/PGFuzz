# PGFUZZ

PGFUZZ is a policy-guided fuzzing framework. You can freely use it to define and find more bugs. By specifying a metric temporal logic (MTL) formula, PGFUZZ is able to mutate inputs related to the formula and to discover more interesting bug cases. Many of the ideas behind PGFUZZ are documented in a paper published at NDSS 2021.

<p>
<a href="https://kimhyungsub.github.io/NDSS21_hskim.pdf"> <img align="right" width="220"  src="https://kimhyungsub.github.io/PGFUZZ_paper_cover.png"> </a>
</p>

The current PGFUZZ version is for ArduPilot. We will upload other verions of PGFUZZ for PX4 and Paparazzi after we finish checking each component's functionality.

## 1. Setup
We assume that you already finished setup for executing ArduPilot, PX4 and Paparazzi on Ubuntu LTS / Debian Linux. <br>
- <a href="https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux" target="_blank"> ArduPilot setup </a>
- <a href="https://docs.google.com/document/d/1P8XIntjK5QtkaPua1yUPTRRZP-TzDdS_rlY7Z-IJGQ8/edit?usp=sharing" target="_blank"> ArduPilot SITL setup </a>
- <a href="https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html" target="_blank"> PX4 setup </a>
- <a href="https://wiki.paparazziuav.org/wiki/NPS" target="_blank"> Paparazzi setup </a>

## 2. Download PGFUZZ
```bash
cd ~
git clone https://github.com/purseclab/PGFUZZ.git pgfuzz
```

## 3. Download a virtual machine image
If you are not used to ArduPilot build environment, I would recommand you to download <a href="https://drive.google.com/file/d/1OFClgdXTS5aeCWzEdLrlOAWfC4VDDbSp/view?usp=sharing" target="_blank"> the virtual machine image</a> and run PGFUZZ.

The image is created on VMware Workstation Player

OS: Ubuntu 18.04

User name: pgfuzz

password: pgfuzz

```bash
cd ~
cd pgfuzz
```

## 4. Bugs discovered by PGFUZZ
If you want to closely look into bug cases discovered by PGFUZZ, please review the below documentation. 
<a href="https://docs.google.com/spreadsheets/d/1zhCx4SzuMZQDMSzBtofIpiqJt2DfjhDlPYxJy1D023M/edit?usp=sharing"> <img align="center" width="760"  src="https://kimhyungsub.github.io/Bug_report.jpg"> </a>

## 5. Demonstration videos
### Case 1
- <a href="https://youtu.be/EiWLCj-pQ7M"> Correct behavior of the parachute </a>
- <a href="https://youtu.be/nhmKE03-bnk"> (i) Buggy behavior of the parachute and (ii) parachute operations after patching the bug</a>  
<br> <b>Q</b>: Why is this case a logic bug? <br>
<b>A</b>: the ArduPilot official documentation states that the following four conditions must hold to deploy a parachute while preserving the drone safety: (1) the motors must be armed, (2) the vehicle must not be in the FLIP or ACRO flight modes, (3) the barometer must show that the
vehicle is not climbing, and (4) the vehicle’s current altitude must be above the CHUTE_ALT_MIN parameter value. <br>
we found that ArduPilot improperly checks the first three requirements. 
This leads to a policy violation where the vehicle deploys the parachute when it is climbing, causing it to crash on the ground. 

### Case 2
- <a href="https://youtu.be/DmCsFHjZV_Q"> Correct behavior of the GPS fail-safe </a>
- <a href="https://youtu.be/gI2BPWIkYTs"> (i) Buggy behavior of the GPS fail-safe and (ii) GPS fail-safe operations after patching the bug</a>
<br> <b>Q</b>: Why is this case a logic bug? <br>
<b>A</b>: The buggy behavior occurs because PX4 developers remove a parameter range check. 
PX4 v1.7.4 forces COM_POS_FS_DELAY parameter to have a value in the valid range. 
Thereafter, it checks whether the GPS fail-safe needs to be triggered. 
However, we found that the code lines to check the COM_POS_FS_DELAY parameter are removed by developers in PX4 v1.9 while updating the fail-safe code snippets. <br>
When a user assigns a negative value to the parameter, it affects the decision to trigger the fail-safe when the current flight mode is ORBIT or the drone is flying into a location. 
Specifically, if the flight mode is not ORBIT or the drone stays at the same location, PX4 correctly triggers the GPS fail-safe. 
This observation makes it difficult for the developers to notice the bug. 
