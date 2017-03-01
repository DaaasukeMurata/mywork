#!/usr/bin/env python
# coding: UTF-8

import sys
import progressbar


def get_time(line):
    if (len(line) < 1):
        return -1
    # print line[:40]
    cols = line.split(',')
    # csvファイル先頭の '%time,field.header..' は無効値
    if cols[0].startswith('%'):
        return -1
    return int(cols[0])


def concat_csv(image_line, servo_line):
    image_cols = image_line.split(',')
    servo_cols = servo_line.split(',')

    # dataの開始位置を取得
    # time, seq, header.stamp, frame_id, height, width, encoding, bigendian, step, data0, data1....
    # 0     1    2             3         4       5      6         7          8     9      10
    image_start = len(image_cols[0] + image_cols[1] + image_cols[2]
                      + image_cols[3] + image_cols[4] + image_cols[5]
                      + image_cols[6] + image_cols[7] + image_cols[8]) + 9

    # dataの開始位置を取得
    # time, data_offset, data0, data1
    # 0     1            2      3
    servo_start = len(servo_cols[0] + servo_cols[1]) + 2

    return servo_line[servo_start:-1] + ',' + image_line[image_start:]


# if __name__ == '__main__':

if (len(sys.argv) != 4):
    print 'usage : python concat_csv_alignment_w_time.py [in_image.csv] [in_servo.bag] [out_file.csv]'
    sys.exit()

in_image_csv = sys.argv[1]
in_servo_csv = sys.argv[2]
out_file_csv = sys.argv[3]

with open(in_image_csv, 'r') as image_csv, open(in_servo_csv, 'r') as servo_csv, open(out_file_csv, 'w') as out_csv:

    # ProgressBar
    image_lines = sum(1 for line in image_csv)
    image_csv.seek(0)
    bar = progressbar.ProgressBar(max_value=image_lines)
    count_lines = 0

    # csvファイル先頭の '%time,field.header..' は読み飛ばしておく
    image_line = image_csv.readline()
    image_time = get_time(image_line)
    count_lines = count_lines + 1
    if (image_time == -1):
        image_line = image_csv.readline()
        image_time = get_time(image_line)
        count_lines = count_lines + 1
    servo_line = servo_csv.readline()
    servo_time = get_time(servo_line)
    if (servo_time == -1):
        servo_line = servo_csv.readline()
        servo_time = get_time(servo_line)

    # image_csv, servo_csvともに時間順に並んでいる
    while image_line:
        # ProgressBar
        bar.update(count_lines)

        while servo_line:
            # imageを受けての、最初のservo値を使用
            if (image_time <= servo_time):
                out_line = concat_csv(image_line, servo_line)
                out_csv.write(out_line)
                break

            servo_line = servo_csv.readline()
            servo_time = get_time(servo_line)

        # servo値がない場合は、データ破棄
        image_line = image_csv.readline()
        image_time = get_time(image_line)
        count_lines = count_lines + 1
