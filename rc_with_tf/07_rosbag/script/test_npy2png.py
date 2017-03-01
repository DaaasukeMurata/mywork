#!/usr/bin/env python
# coding: UTF-8

import sys
import numpy as np
from PIL import Image

# if __name__ == '__main__':

if (len(sys.argv) != 3):
    print 'usage : npy_test.py [in_file.npy] [out_image.png]'
    sys.exit(1)

in_file_npy = sys.argv[1]
out_image_png = sys.argv[2]

depth = 2
height = 60
width = 160
image_bytes = depth * height * width

CHECK_INDEX = 400

with open(in_file_npy, 'rb') as in_npy:
    data = np.load(in_npy)

    # 画像データ、1次元 -> [height, width, depth]に
    image_buffer = data[CHECK_INDEX][2:]
    byte_buffer = np.frombuffer(image_buffer, dtype=np.uint8)
    reshaped_array = np.reshape(byte_buffer, [height, width, depth])

    # rgbのbが足らないため、dummyを付与
    dummy_array = np.zeros((60, 160, 1), np.uint8)
    image_array = np.dstack((reshaped_array, dummy_array))

    imageshow = Image.fromarray(image_array.astype(np.uint8))

    with open(out_image_png, 'wb') as out_png:
        imageshow.save(out_png, format='png')
