#!/usr/bin/env python
# coding: UTF-8
import sys
import numpy as np


def main():
    if (len(sys.argv) != 3):
        print 'usage : python remove_dim3_npy.py [in.npy] [out.npy]'
        sys.exit(1)

    in_npy = sys.argv[1]
    out_npy = sys.argv[2]

    depth=3
    height=60
    width=160

    with open(in_npy, 'rb') as in_f, open(out_npy, 'wb') as out_f:
        in_buffers = np.load(in_f)
        out_array = None

        for buffer in in_buffers:
            image_buffer = buffer[2:]

            # [height, width, depth]の形にして、dim3を削除
            image_array = np.reshape(image_buffer, [height, width, depth])
            dim1, dim2, dim3 = np.dsplit(image_array, 3)
            image_array = np.dstack((dim1, dim2))

            # 1次元に戻して、servoと再結合
            image_buf = image_array.flatten()
            concat_buf = np.hstack((buffer[:2], image_buf))
            # print concat_buf.shape

            # 出力用のarrayに加える
            if out_array is None:
                out_array = concat_buf
            else:
                out_array = np.vstack((out_array, concat_buf))

        np.save(out_f, out_array)

if __name__ == '__main__':
    main()
