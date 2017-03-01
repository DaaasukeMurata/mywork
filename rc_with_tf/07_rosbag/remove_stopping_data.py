#!/usr/bin/env python
# coding: UTF-8
import sys
import numpy as np
import progressbar


def main():
    if (len(sys.argv) != 3):
        print 'usage : python remove_straight_data.py [in.npy] [out.npy]'
        sys.exit(1)

    in_npy = sys.argv[1]
    out_npy = sys.argv[2]

    out_array = None

    print(in_npy)

    with open(in_npy, 'rb') as f:
        in_array = np.load(f)
        print(in_array.shape)

        # progressbar
        max_lines = in_array.shape[0]
        bar = progressbar.ProgressBar(max_value=max_lines)
        now_lines = 0

        for array in in_array:
            # ProgressBar
            now_lines = now_lines + 1
            bar.update(now_lines)

            if (array[1] != 90):
                # print array.shape
                reshaped = array.reshape(1, array.shape[0])
                if out_array is None:
                    out_array = reshaped
                else:
                    out_array = np.concatenate((out_array, reshaped))

    with open(out_npy, 'wb') as f:
        print(out_array.shape)
        np.save(f, out_array)

if __name__ == '__main__':
    main()
