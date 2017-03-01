#!/usr/bin/env python
# coding: UTF-8

import sys
import rosbag
from sensor_msgs.msg import Image
import progressbar

# if __name__ == '__main__':

if (len(sys.argv) != 4):
    print 'usage : python convert_csv2rosbag_image.py [in_image.csv] [out_file.bag] [out_topicname]'
    sys.exit()

in_image_csv = sys.argv[1]
out_file_bag = sys.argv[2]
out_topicname = sys.argv[3]

with open(in_image_csv, 'r') as csv_file, rosbag.Bag(out_file_bag, 'w') as outbag:
    # ProgressBar
    num_lines = sum(1 for line in open(in_image_csv))
    bar = progressbar.ProgressBar(max_value=num_lines)
    count_lines = 0

    csv_line = csv_file.readline()

    while csv_line:
        # ProgressBar
        count_lines = count_lines + 1
        bar.update(count_lines)

        # time, seq, header.stamp, frame_id, height, width, encoding, bigendian, step, data0, data1....
        # 0     1    2             3         4       5      6         7          8     9      10
        cols = csv_line.split(',')

        # csvファイル先頭の '%time,field.header..' を読み飛ばす
        if cols[0].startswith('%'):
            csv_line = csv_file.readline()
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
        # リスト作成し、設定
        wk_list = []
        for index, item in enumerate(cols[9:]):
            wk_list.append(int(item))
        img_msg.data = wk_list

        outbag.write(out_topicname, img_msg)
        csv_line = csv_file.readline()


# with rosbag.Bag('test_time.bag', 'w') as outbag, rosbag.Bag(out_file_bag) as tmpbag:
#     for topic, msg, t in rosbag.Bag(in_timestamp_bag).read_messages():
#         tmpmsg = tmpbag.read_messages()
#         # This also replaces tf timestamps under the assumption
#         # that all transforms in the message share the same timestamp
#         if topic == "/tf" and msg.transforms:
#             outbag.write(out_topicname, tmpmsg, msg.transforms[0].header.stamp)
#         else:
#             outbag.write(out_topicname, tmpmsg, msg.header.stamp if msg._has_header else t)
