#!/usr/bin/env python
# coding: UTF-8

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import os
from PIL import Image

import numpy as np


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
        self.image_array = np.reshape(byte_buffer, [self.height, self.width, self.depth])
        self.image_array = self.image_array.astype(np.float32)


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
        result = RcImageRecord()

        label_bytes = 2
        image_bytes = result.height * result.width * result.depth
        record_bytes = label_bytes + image_bytes

        label_buffer = self.bytes_array[index][:label_bytes]
        # image_buffer = self.bytes_array[index][label_bytes:label_bytes + image_bytes]
        image_buffer = self.bytes_array[index][label_bytes:]

        result.set_label(label_buffer)
        result.set_image(image_buffer)

        return result


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
