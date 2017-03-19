#!/bin/bash

function image_process () {
  sh_path=`dirname $0`
  image_processed_topic="/image_processed"
  fname_raw_image=$1
  fname_processed_image=$2

  # bag
  echo "[CreTrD] ${sh_path}/script/create_rosbag_image_processed.sh ${fname_raw_image}.bag  ${fname_processed_image}.bag"
  ${sh_path}/script/create_rosbag_image_processed.sh ${fname_raw_image}.bag ${fname_processed_image}.bag

  # bag -> csv
  echo "[CreTrD] rostopic echo -b ${fname_processed_image}.bag -p ${image_processed_topic} > ${fname_processed_image}.csv"
  rostopic echo -b ${fname_processed_image}.bag -p ${image_processed_topic} > ${fname_processed_image}.csv
}

function create_train_data () {
  sh_path=`dirname $0`
  # DEFINE
  image_raw_topic="/usb_cam_node/image_raw"
  image_processed_topic="/image_processed"
  servo_topic='/servo'
  arg1=$1
  arg2=$2
  wk_fname="${arg1%.*}"    # 拡張子除去 "dir/filename.bag" -> "dir/filename"
  fname="${wk_fname##*/}"  # pathを削除 "dir/filename" -> "filename"
  outdir=${arg2%/}           # directory最後の"/"は削除  "dir/dir/" -> "dir/dir"
  fname_raw_image="${outdir}/${fname}_raw_image"
  fname_processed_image="${outdir}/${fname}_processed_image"
  fname_align_image="${outdir}/${fname}_align_image"
  fname_servo="${outdir}/${fname}_servo"
  fname_concat="${outdir}/${fname}_concat"
  fname_learn_data="${outdir}/${fname}_train_data"

  # [raw image]
  # bag
  echo "[CreTrD] rosbag filter $1 ${fname_raw_image}.bag topic == ${image_raw_topic}"
  rosbag filter $1 ${fname_raw_image}.bag "topic == '${image_raw_topic}'"

  # bag -> csv
  echo "[CreTrD] ${sh_path}/script/bag2time.py -t ${image_raw_topic} ${fname_raw_image}.bag > ${fname_raw_image}.csv"
  python ${sh_path}/script/bag2time.py -t ${image_raw_topic} ${fname_raw_image}.bag > ${fname_raw_image}.csv


  # [image processing]
  for i in `seq 0 9`
  do
    image_process ${fname_raw_image} ${fname_processed_image}

    # check data
    raw_num=$( wc -l < ${fname_raw_image}.csv )
    processed_num=$( wc -l < ${fname_processed_image}.csv )
    if [ ${raw_num} -eq ${processed_num} ]; then
      # success.
      break
    fi

    # fail. delete files and retry
    echo "[CreTrD][ERR] unmatch column in raw_image and processed_image"
    echo "         ${raw_num} ${fname_raw_image}.csv"
    echo "         ${processed_num} ${fname_processed_image}.csv"
    if [ -e ${fname_processed_image}.bag ]; then
      rm ${fname_processed_image}.bag
    fi
    if [ -e ${fname_processed_image}.csv ]; then
      rm ${fname_processed_image}.csv
    fi
  done



  # [servo]
  # bag -> csv
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

  # echo "[CreTrD] python ${sh_path}/script/convert_csv2bin_uint8.py ${fname_concat}.csv ${fname_learn_data}_wk.npy"
  # python ${sh_path}/script/convert_csv2bin_uint8.py ${fname_concat}.csv ${fname_learn_data}_wk.npy
  # npyの３次元目は今何も入れていないため削除（1:edge, 2:DetectLine, ）
  # echo "[CreTrD] python ${sh_path}/script/remove_dim3_npy.py ${fname_learn_data}_wk.npy ${fname_learn_data}.npy"
  # python ${sh_path}/script/remove_dim3_npy.py ${fname_learn_data}_wk.npy ${fname_learn_data}.npy


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
  if [ -e ${fname_learn_data}_wk.npy ]; then
    rm ${fname_learn_data}_wk.npy
  fi
}



if [ $# -ne 2 ]; then
  echo "[CreTrD] [usage] $ ./create_learn_data.sh [in_directory or xxx.bag] [outdir]"
  exit 1
fi

if [ ! -d $2 ]; then
  echo "[CreTrD] Out directory is not exist. dir:$2"
  exit 1
fi

if [ -d $1 ]; then
  for file in `\find $1 -maxdepth 1 -name '*.bag'`; do
    create_train_data $file $2
  done
  echo "[CreTrD] finished create_learn_data.sh"
  echo "Output Directory:$2"

elif [ -f $1 ]; then
  create_train_data $1 $2
  echo "[CreTrD] finished create_learn_data.sh"
  echo "Output Directory:$2"

else
  echo "[CreTrD] [usage] $ ./create_learn_data.sh [in_directory or xxx.bag] [outdir]"
  exit 1
fi
