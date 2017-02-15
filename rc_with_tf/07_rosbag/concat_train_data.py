#!/usr/bin/env python
# coding: UTF-8
import glob
import sys
import numpy as np


def main():
    if (len(sys.argv) != 3):
        print 'usage : python concat_train_data.py [in_directory_path] [out.npy]'
        sys.exit(1)

    in_directory = sys.argv[1]
    out_npy = sys.argv[2]

    files = glob.glob(in_directory + '*.npy')
    out_array = None

    for file in files:
        print(file)
        with open(file, 'rb') as f:
            in_array = np.load(f)
            if out_array is None:
                out_array = in_array
            else:
                out_array = np.concatenate((out_array, in_array))

    with open(out_npy, 'wb') as f:
        print(out_array.shape)
        np.save(f, out_array)

if __name__ == '__main__':
    main()
