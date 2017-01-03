#!/bin/bash

if [ $# -ne 1 ]; then
  echo "[usage] $ ./create_learn_data.sh [in_alltopic.bag]"
  exit 1
fi

# DEFINE
image_raw_topic="/image_raw"
image_processed_topic="/image_processed"
servo_topic='/servo'
# xxxx.bagの拡張子除去
arg1=$1
fname="${arg1%.*}"
fname_raw_image="${fname}_raw_image"
fname_processed_image="${fname}_processed_image"
fname_align_image="${fname}_align_image"
fname_servo="${fname}_servo"
fname_concat="${fname}_concat"
fname_learn_data="${fname}_learn_data"

# image processing
echo "rosbag filter $1 ${fname_raw_image}.bag topic == ${image_raw_topic}"
rosbag filter $1 ${fname_raw_image}.bag "topic == '${image_raw_topic}'"

echo "./script/create_rosbag_image_processed.sh ${fname_raw_image}.bag  ${fname_processed_image}.bag"
./script/create_rosbag_image_processed.sh ${fname_raw_image}.bag  ${fname_processed_image}.bag


# bag -> csv 生成
echo "rostopic echo -b ${fname_raw_image}.bag -p ${image_raw_topic} > ${fname_raw_image}.csv"
rostopic echo -b ${fname_raw_image}.bag -p ${image_raw_topic} > ${fname_raw_image}.csv

echo "rostopic echo -b ${fname_processed_image}.bag -p ${image_processed_topic} > ${fname_processed_image}.csv"
rostopic echo -b ${fname_processed_image}.bag -p ${image_processed_topic} > ${fname_processed_image}.csv

echo "rostopic echo -b $1 -p ${servo_topic} > ${fname_servo}.csv"
rostopic echo -b $1 -p ${servo_topic} > ${fname_servo}.csv


# image_raw, image_processedの時間をあわせる
echo "python ./script/align_time_image_csv.py ${fname_raw_image}.csv ${fname_processed_image}.csv ${fname_align_image}.csv"
python ./script/align_time_image_csv.py ${fname_raw_image}.csv ${fname_processed_image}.csv ${fname_align_image}.csv


# servoとimageの結合
echo "python ./script/concat_csv_alignment_w_time.py ${fname_align_image}.csv ${fname_servo}.csv ${fname_concat}.csv"
python ./script/concat_csv_alignment_w_time.py ${fname_align_image}.csv ${fname_servo}.csv ${fname_concat}.csv


# csv -> bin
echo "python ./script/convert_csv2bin_uint8.py ${fname_concat}.csv ${fname_learn_data}.npy"
python ./script/convert_csv2bin_uint8.py ${fname_concat}.csv ${fname_learn_data}.npy


# delete temporary_files
echo "delete temporary_files"
if [ -e ${fname_raw_image}.bag ]; then
  rm ${fname_raw_image}.bag
fi
if [ -e ${fname_raw_image}.csv ]; then
  rm ${fname_raw_image}.csv
fi
if [ -e ${fname_processed_image}.bag ]; then
  rm ${fname_processed_image}.bag
fi
if [ -e ${fname_processed_image}.csv ]; then
  rm ${fname_processed_image}.csv
fi
if [ -e ${fname_align_image}.csv ]; then
  rm ${fname_align_image}.csv
fi
if [ -e ${fname_servo}.csv ]; then
  rm ${fname_servo}.csv
fi
if [ -e ${fname_concat}.csv ]; then
  rm ${fname_concat}.csv
fi

echo "finished create_learn_data.sh"

