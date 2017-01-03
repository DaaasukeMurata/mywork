#!/bin/bash

if [ $# -ne 1 ]; then
  echo "[usage] $ ./filter_rosbag_image.sh [in_alltopic.bag]"
  exit 1
fi

# xxxx.bagの拡張子除去
arg1=$1
fname="${arg1%.*}"

# image_rawをbagへ出力。topicは指定しない
echo "rosbag filter $1 ${fname}_image_raw.bag topic == /image_raw"
rosbag filter $1 ${fname}_image_raw.bag "topic == '/image_raw'"
