#!/usr/bin/env python
# coding: UTF-8

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import os
from PIL import Image

import numpy as np


class LineInfo(object):

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.piangle = self.__get_piangle()

    def __get_piangle(self):
        # y = tan(θ) * x + b
        vy = self.y2 - self.y1
        vx = self.x2 - self.x1
        return math.atan2(vy, vx) / math.pi


class RcImageRecord(object):
    width = 160
    height = 60
    depth = 2

    def set_label(self, label_bytes):
        self.steer_array = np.zeros(180)
        self.steer_array[label_bytes[0]] = 1
        self.speed_array = np.zeros(180)
        self.speed_array[label_bytes[1]] = 1

    def set_image(self, image_bytes):
        byte_buffer = np.frombuffer(image_bytes, dtype=np.int8)

        # dim1,2のみ画像、dim3はline info
        image_array = np.reshape(byte_buffer, [height, width, depth])
        dim1, dim2, dim3 = np.dsplit(image_array, 3)

        # 画像
        image_array = np.dstack((dim1, dim2))
        self.image_array = image_array.astype(np.float32)

        # line info
        self.f_line = LineInfo(dim3[0, 0, 0], dim3[0, 1, 0], dim3[0, 2, 0], dim3[0, 3, 0])
        self.l_line = LineInfo(dim3[1, 0, 0], dim3[1, 1, 0], dim3[1, 2, 0], dim3[1, 3, 0])


class RcImageReader(object):

    def __init__(self, filename):
        if not os.path.exists(filename):
            print(filename + ' is not exist')
            return

        with open(filename, 'rb') as npy:
            self.bytes_array = np.load(npy)

    def shuffle(self):
        np.random.shuffle(self.bytes_array)

    def read(self, index):
        record = RcImageRecord()

        label_bytes = 2
        image_bytes = record.height * record.width * record.depth
        record_bytes = label_bytes + image_bytes

        label_buffer = self.bytes_array[index][:label_bytes]
        # image_buffer = self.bytes_array[index][label_bytes:label_bytes + image_bytes]
        image_buffer = self.bytes_array[index][label_bytes:]

        record.set_label(label_buffer)
        record.set_image(image_buffer)

        return record


def test(filename, index=10):
    reader = RcImageReader(sys.argv[1])
    print(reader.bytes_array.shape)

    record = reader.read(index)
    print(record.steer_array)

    image = record.image_array.astype(np.uint8)
    print(image.shape)

    if record.depth == 2:
        # rgbのbが足らないため、dummyを付与
        dummy_array = np.zeros((60, 160, 1), np.uint8)
        image = np.dstack((image, dummy_array))

    imageshow = Image.fromarray(image)

    with open('reader_test.png', mode='wb') as out:
        imageshow.save(out, format='png')


if __name__ == '__main__':
    test(sys.argv[1])
