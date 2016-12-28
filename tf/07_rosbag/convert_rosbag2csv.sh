#!/bin/bash

if [ $# -ne 2 ]; then
  echo "[usage] $ ./convert_rosbag2csv.sh [ros.bag] [topicname]"
  exit 1
fi

# xxxx.bagの拡張子除去
arg1=$1
fname="${arg1%.*}"

# csvへ出力。topicは指定しない
echo "rostopic echo -b $1 -p $2 > ${fname}.csv"
rostopic echo -b $1 -p $2 > ${fname}.csv


