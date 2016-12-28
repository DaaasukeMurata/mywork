#!/bin/bash

IMAGE_PROCESS_CMD="python ../05_image_process/image_process.py --disable-gui image:=/image_raw"

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

sleep 1s

# /image_rawの配信開始
rosbag play $1

# 起動したソフトの終了
kill -SIGINT ${PID_image_process_cmd}
kill -SIGINT `ps ho pid --ppid=${PID_rosbag_record}`





