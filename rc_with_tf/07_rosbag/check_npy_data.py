#!/usr/bin/env python
# coding: UTF-8
import sys
import numpy as np


def main():
    if (len(sys.argv) != 2):
        print 'usage : python check_steer_num.py [in.npy]'
        sys.exit(1)

    in_npy = sys.argv[1]

    count_array = [0] * 180

    print(in_npy)
    with open(in_npy, 'rb') as f:
        in_array = np.load(f)
        for array in in_array:
            val = array[0]      # 0:steer, 1:speed
            count_array[val] += 1

        for index in range(len(count_array)):
            print('%3d:%5d   ' % (index, count_array[index])),
            if ((index + 1) % 5 == 0):
                print

        print
        print('file :{0}'.format(in_npy))
        print('shape:{0}'.format(in_array.shape))

if __name__ == '__main__':
    main()
