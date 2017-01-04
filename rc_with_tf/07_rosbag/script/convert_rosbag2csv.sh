#!/bin/bash

if [ $# -ne 3 ]; then
  echo "[usage] $ ./convert_rosbag2csv.sh [in_ros.bag] [out.csv] [topicname]"
  exit 1
fi

# csvへ出力。
echo "rostopic echo -b $1 -p $3 > $2"
rostopic echo -b $1 -p $3 > $2


