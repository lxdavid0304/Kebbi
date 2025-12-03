#!/bin/bash
conda activate o3d
cd /home/jetson/Desktop/Kebbi
cd tof_maixsense
while [ 1 ]; do python Human_Detection_Depth_Camera.py;sleep 1; done&
cd ../legacy_capture_scripts
while [ 1 ]; do python dynamic_world_occ.py;sleep 1; done

