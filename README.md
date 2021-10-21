# PGFUZZ

PGFUZZ is a policy-guided fuzzing framework. You can freely use it to define and find more bugs. By specifying a metric temporal logic (MTL) formula, PGFUZZ is able to mutate inputs related to the formula and to discover more interesting bug cases. Many of the ideas behind PGFUZZ are documented in a paper published at NDSS 2021.

<p>
<a href="https://kimhyungsub.github.io/NDSS21_hskim.pdf"> <img align="right" width="220"  src="https://kimhyungsub.github.io/PGFUZZ_paper_cover.png"> </a>
</p>

The current PGFUZZ version is for ArduPilot. We will upload other verions of PGFUZZ for PX4 and Paparazzi after we finish checking each component's functionality.

## Setup
We assume that you already finished setup for executing ArduPilot, PX4 and Paparazzi on Ubuntu LTS / Debian Linux. <br>
- <a href="https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux" target="_blank"> ArduPilot setup </a>
- <a href="https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html" target="_blank"> PX4 setup </a>
- <a href="https://wiki.paparazziuav.org/wiki/NPS" target="_blank"> Paparazzi setup </a>

## Download PGFUZZ
```bash
cd ~
git clone https://github.com/purseclab/PGFUZZ.git pgfuzz
```

## Download a virtual machine image
If you are not used to ArduPilot build environment, I would recommand you to download <a href="https://drive.google.com/file/d/1OFClgdXTS5aeCWzEdLrlOAWfC4VDDbSp/view?usp=sharing" target="_blank"> the virtual machine image</a> and run PGFUZZ.

The image is created on VMware Workstation Player

OS: Ubuntu 18.04

User name: pgfuzz

password: pgfuzz

```bash
cd ~
cd pgfuzz
```

## Bugs discovered by PGFUZZ
If you want to closely look into bug cases discovered by PGFUZZ, please review the below documentation. 
<a href="https://docs.google.com/spreadsheets/d/1zhCx4SzuMZQDMSzBtofIpiqJt2DfjhDlPYxJy1D023M/edit?usp=sharing"> <img align="center" width="760"  src="https://kimhyungsub.github.io/Bug_report.jpg"> </a>.


