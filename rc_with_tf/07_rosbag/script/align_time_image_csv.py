#!/usr/bin/env python
# coding: UTF-8

import sys
import progressbar

# if __name__ == '__main__':

if (len(sys.argv) != 4):
    print 'usage : python align_time_image.py [in_timebase.csv] [in_image.csv] [out_file.csv]'
    sys.exit()

in_time_csv = sys.argv[1]
in_image_csv = sys.argv[2]
out_file_csv = sys.argv[3]

with open(in_time_csv, 'r') as time_csv, open(in_image_csv, 'r') as image_csv, open(out_file_csv, 'w') as out_csv:

    # sample数が一致するかチェック
    time_lines = sum(1 for line in time_csv)
    time_csv.seek(0)
    image_lines = sum(1 for line in image_csv)
    image_csv.seek(0)
    if (time_lines != image_lines):
        print '[ERROR] unmatched num of lines'
        sys.exit(1)

    # ProgressBar
    bar = progressbar.ProgressBar(max_value=time_lines)
    count_lines = 0

    time_line = time_csv.readline()
    image_line = image_csv.readline()
    count_lines = count_lines + 1

    while time_line:
        # ProgressBar
        bar.update(count_lines)

        # time, seq, header.stamp, frame_id, height, width, encoding, bigendian, step, data0, data1....
        # 0     1    2             3         4       5      6         7          8     9      10
        time_cols = time_line.split(',')
        image_cols = image_line.split(',')

        # header.stampをin_time_csv, それ以降をin_image_csvとする
        time_end = len(time_cols[0]) + 1
        image_start = len(image_cols[0]) + 1
        out_line = time_line[:time_end] + image_line[image_start:]
        out_csv.write(out_line)

        time_line = time_csv.readline()
        image_line = image_csv.readline()
        count_lines = count_lines + 1
