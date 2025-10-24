#!/bin/bash

#CAM_LIST can be for example:
#cam0
#cam0,cam1
#cam0,cam1,cam2
#cam0,cam1,cam2,cam3
#cam1,cam2
#cam0,cam3

#CAM_MODEL can be:
#imx219 or ov5640

CAM_LIST="cam0"
CAM_MODEL="ov5640"
PYV4L2_PATH="/home/analog/Workspace/pyv4l2"
DT_MODEL_PATH="/proc/device-tree/model"

if [[ $(cat $DT_MODEL_PATH | tr -d '\0') =~ "Raspberry Pi 4" ]]; then
  echo "Running on RPI4"
  RPI_MODEL="rpi4"
elif [[ $(cat $DT_MODEL_PATH | tr -d '\0') =~ "Raspberry Pi 5" ]]; then
  echo "Running on RPI5"
  RPI_MODEL="rpi5"
else
  echo "Unknown platform"
  exit 0
fi

$PYV4L2_PATH/utils/cam.py -c -p "$RPI_MODEL"-gmsl-"$CAM_MODEL":$CAM_LIST
