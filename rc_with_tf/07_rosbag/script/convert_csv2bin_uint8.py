#!/usr/bin/env python
# coding: UTF-8

import sys
import numpy as np

# if __name__ == '__main__':

if (len(sys.argv) != 3):
    print 'usage : python convert_csv2bin.py [in_concat.csv] [out_file.npy]'
    sys.exit(1)

in_concat_csv = sys.argv[1]
out_file_npy = sys.argv[2]

with open(in_concat_csv, 'r') as in_csv, open(out_file_npy, 'wb') as out_npy:
    data = np.loadtxt(in_csv, delimiter=",", dtype=np.uint8)
    np.save(out_npy, data)
