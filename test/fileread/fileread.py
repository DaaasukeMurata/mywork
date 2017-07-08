#!/usr/bin/env python
# coding: utf-8

import sys
import time

def main():
    argvs = sys.argv
    if len(argvs) < 2:
        print('[usage]python fileread.py [file]')
        sys.exit(0)

    with open(argvs[1], 'r') as f:
        # for line in f:
        #     print(line),
        while True:
            line = f.readline()
            sys.stdout.write(line)
            time.sleep(1)


if __name__ == '__main__':

    main()
