#!/bin/bash

IMAGE_PROCESS_CMD="rosrun rc_image_w_tf rc_line_detect.py _gui:=false _dim:=2 _image:=/usb_cam_node/image_raw"

if [ $# -ne 2 ]; then
  echo "[usage] $ ./create_rosbag_image_processed.sh [image_raw.bag] [output filename(image_processed.bag)]"
  exit 1
fi

# /image_processedのrosbag記録開始
rosbag record -O $2 /image_processed &
PID_rosbag_record=$!

# run image_process
${IMAGE_PROCESS_CMD} &
PID_image_process_cmd=$!

sleep 2s

# /image_rawの配信開始
rosbag play $1

# 起動したプログラムの終了
kill -SIGINT ${PID_image_process_cmd}
kill -SIGINT `ps ho pid --ppid=${PID_rosbag_record}`
