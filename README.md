# NMPC Adaptive Control of a Micro 3D-Helicopter

## Motivation
This is an implementation of the L1 adaptive model predictive controller used by ETH Zurich onto an autonomous nano 3D helicopter (http://rpg.ifi.uzh.ch/docs/Arxiv21_L1_Drew.pdf). We developed this repository to track the progess of our implimentation of the idea and document the steps needed to get the project operational

# Simulation

## Requirements
Simulation enviornment is created in MATLAB and requires the ACADO Matlab Interface. Full installation instructions can be found at https://acado.github.io/matlab_overview.html but they are also listed here:

1: Download the acado toolkit from: https://github.com/acado/acado

2: Open the MATLAB editor and in the command window:
````bash
mex -setup;
````
If matlab returns:
````bash
Please choose your compiler for building external interface (MEX) files:
Would you like mex to locate installed compilers [y]/n?
````
type "y" and hit enter

3: In the Matlab command window, navigate to the interfaces/matlab/ folder in the acado toolkit download.

4:
````bash
make clean all;
````

ACADO for matlab will now build

5: After the build has finished run the command:
''''bash
savepath;
''''

to save the build path to the matlab install

  
# ROS Implimentation
