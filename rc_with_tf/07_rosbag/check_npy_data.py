#!/usr/bin/env python
# coding: UTF-8
import sys
import numpy as np


def print_list(list):
    ''' output sample
    steer:
    [  0-  9:    0]   [ 10- 19:    0]   [ 20- 29:    0]
    [ 30- 39: 3862]   [ 40- 49:  122]   [ 50- 59:  147]
    [ 60- 69:  440]   [ 70- 79:  693]   [ 80- 89:  889]
    [ 90- 90:15631]
    [ 91-100:  241]   [101-110:  201]   [111-120:  316]
    [121-130:   86]   [131-140:  107]   [141-150:  674]
    [151-160:    0]   [161-170:    0]   [171-180:    0]
    '''
    for i in [0, 10, 20, 30, 40, 50, 60, 70, 80]:
        print('  [%3d-%3d:%5d]  ' % (i, i + 9, sum(list[i:i + 10]))),
        if i in [20, 50, 80, 110, 140, 170]:
            print
    print('  [%3d-%3d:%5d]  ' % (90, 90, list[90]))
    for i in [90, 100, 110, 120, 130, 140, 150, 160, 170]:
        print('  [%3d-%3d:%5d]  ' % (i + 1, i + 10, sum(list[i + 1:i + 11]))),
        if i in [20, 50, 80, 110, 140, 170]:
            print


def main():
    if (len(sys.argv) != 2):
        print 'usage : python check_steer_num.py [in.npy]'
        sys.exit(1)

    in_npy = sys.argv[1]

    depth = 3
    height = 60
    width = 160

    count_steer = [0] * 180
    count_speed = [0] * 180
    count_fline = 0
    count_lline = 0

    with open(in_npy, 'rb') as f:
        in_array = np.load(f)

        print
        print('file :{0}'.format(in_npy))
        print('shape:{0}'.format(in_array.shape))

        for array in in_array:
            val = array[0]      # 0:steer, 1:speed
            count_steer[val] += 1
            val = array[1]      # 0:steer, 1:speed
            count_speed[val] += 1

            # count line meta
            image_buffer = array[2:]
            image_array = np.reshape(image_buffer, [height, width, depth])
            if image_array[0, 0, 2] != 0:
                count_fline += 1
            if image_array[1, 0, 2] != 0:
                count_lline += 1

        print('frame of detecting FRONT line: %4d / %d' % (count_fline, in_array.shape[0]))
        print('frame of detecting LEFT  line: %4d / %d' % (count_lline, in_array.shape[0]))

        print
        print('steer:')
        print_list(count_steer)

        print
        print('speed:')
        print_list(count_speed)


if __name__ == '__main__':
    main()
