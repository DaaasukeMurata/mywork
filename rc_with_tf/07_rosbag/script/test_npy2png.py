# -*- coding: utf-8 -*-

import sys
import numpy as np
from PIL import Image

# if __name__ == '__main__':

if (len(sys.argv) != 3):
    print 'usage : npy_test.py [in_file.npy] [out_image.png]'
    sys.exit(1)

in_file_npy = sys.argv[1]
out_image_png = sys.argv[2]

depth = 1
height = 60
width = 80
image_bytes = depth * height * width

with open(in_file_npy, 'rb') as in_npy:
    data = np.load(in_npy)
    image_buffer = data[0][2:image_bytes + 2]

    byte_buffer = np.frombuffer(image_buffer, dtype=np.uint8)
    # reshaped_array = np.reshape(byte_buffer, [depth, height, width])
    reshaped_array = np.reshape(byte_buffer, [height, width])
    # byte_array = np.transpose(reshaped_array, [1, 2, 0])
    byte_array = np.transpose(reshaped_array, [0, 1])

    imageshow = Image.fromarray(byte_array.astype(np.uint8))

    with open(out_image_png, 'wb') as out_png:
        imageshow.save(out_png, format='png')
