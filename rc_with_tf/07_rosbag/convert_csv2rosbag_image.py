# -*- coding: utf-8 -*-

import sys
import rosbag
from sensor_msgs.msg import Image
import progressbar

sys.argv

# if __name__ == '__main__':

with rosbag.Bag('test.bag', 'w') as bag, open('image_processed.csv', 'r') as csv_file:

    # ProgressBar
    num_lines = sum(1 for line in open('image_processed.csv'))
    bar = progressbar.ProgressBar(max_value=num_lines)
    count_lines = 0

    line = csv_file.readline()

    while line:
        # ProgressBar
        count_lines = count_lines + 1
        bar.update(count_lines)

        cols = line.split(',')
        # time, seq, header.stamp, frame_id, height, width, encoding, bigendian, step, data0, data1....
        # 0     1    2             3         4       5      6         7          8     9      10

        # csvファイル先頭の '%time,field.header..' を読み飛ばす
        if cols[0].startswith('%'):
            line = csv_file.readline()
            continue

        # ROSのImage配信形式に変換
        img_msg = Image()
        img_msg.header.seq = int(cols[1])
        img_msg.header.stamp.secs = int(cols[2])
        img_msg.header.frame_id = cols[3]
        img_msg.height = int(cols[4])
        img_msg.width = int(cols[5])
        img_msg.encoding = cols[6]
        img_msg.is_bigendian = int(cols[7])
        img_msg.step = int(cols[8])
        # img_msg.dataは空のリスト
        # リスト作成し、割り当て
        wk_list = []
        for index, item in enumerate(cols[9:]):
            wk_list.append(int(item))
        img_msg.data = wk_list

        bag.write('/image_raw', img_msg)

        line = csv_file.readline()
