#!/bin/bash

function create_train_data () {
  sh_path=`dirname $0`
  # DEFINE
  image_raw_topic="/usb_cam_node/image_raw"
  image_processed_topic="/image_processed"
  servo_topic='/servo'
  arg1=$1
  fname="${arg1%.*}"    # xxxx.bagの拡張子除去
  fname_raw_image="${fname}_raw_image"
  fname_processed_image="${fname}_processed_image"
  fname_align_image="${fname}_align_image"
  fname_servo="${fname}_servo"
  fname_concat="${fname}_concat"
  fname_learn_data="${fname}_train_data"

  # image processing
  echo "[CreTrD] rosbag filter $1 ${fname_raw_image}.bag topic == ${image_raw_topic}"
  rosbag filter $1 ${fname_raw_image}.bag "topic == '${image_raw_topic}'"

  echo "[CreTrD] ${sh_path}/script/create_rosbag_image_processed.sh ${fname_raw_image}.bag  ${fname_processed_image}.bag"
  ${sh_path}/script/create_rosbag_image_processed.sh ${fname_raw_image}.bag  ${fname_processed_image}.bag


  # bag -> csv 生成
  echo "[CreTrD] rostopic echo -b ${fname_raw_image}.bag -p ${image_raw_topic} > ${fname_raw_image}.csv"
  rostopic echo -b ${fname_raw_image}.bag -p ${image_raw_topic} > ${fname_raw_image}.csv

  echo "[CreTrD] rostopic echo -b ${fname_processed_image}.bag -p ${image_processed_topic} > ${fname_processed_image}.csv"
  rostopic echo -b ${fname_processed_image}.bag -p ${image_processed_topic} > ${fname_processed_image}.csv

  echo "[CreTrD] rostopic echo -b $1 -p ${servo_topic} > ${fname_servo}.csv"
  rostopic echo -b $1 -p ${servo_topic} > ${fname_servo}.csv


  # image_raw, image_processedの時間をあわせる
  echo "[CreTrD] python ${sh_path}/script/align_time_image_csv.py ${fname_raw_image}.csv ${fname_processed_image}.csv ${fname_align_image}.csv"
  python ${sh_path}/script/align_time_image_csv.py ${fname_raw_image}.csv ${fname_processed_image}.csv ${fname_align_image}.csv


  # servoとimageの結合
  echo "[CreTrD] python ${sh_path}/script/concat_csv_alignment_w_time.py ${fname_align_image}.csv ${fname_servo}.csv ${fname_concat}.csv"
  python ${sh_path}/script/concat_csv_alignment_w_time.py ${fname_align_image}.csv ${fname_servo}.csv ${fname_concat}.csv


  # csv -> bin
  echo "[CreTrD] python ${sh_path}/script/convert_csv2bin_uint8.py ${fname_concat}.csv ${fname_learn_data}.npy"
  python ${sh_path}/script/convert_csv2bin_uint8.py ${fname_concat}.csv ${fname_learn_data}.npy


  # delete temporary_files
  echo "[CreTrD] delete temporary_files"
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
}



if [ $# -ne 1 ]; then
  echo "[CreTrD] [usage] $ ./create_learn_data.sh [in_directory or xxx.bag]"
  exit 1
fi

if [ -d $1 ]; then
  for file in `\find $1 -maxdepth 1 -name '*.bag'`; do
    create_train_data $file
  done
  echo "[CreTrD] finished create_learn_data.sh"

elif [ -f $1 ]; then
  create_train_data $1
  echo "[CreTrD] finished create_learn_data.sh"

else
  echo "[CreTrD] [usage] $ ./create_learn_data.sh [in_directory or xxx.bag]"
  exit 1
fi
