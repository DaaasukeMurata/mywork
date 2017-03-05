#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np

import tensorflow as tf

from model import CNNModel
from reader import RcImageReader

from PIL import Image

# define
IMG_DIM = 2
EVAL_FILE = os.path.abspath(os.path.dirname(__file__)) + '/data/eval.npy'
CKPT_PATH = os.path.abspath(os.path.dirname(__file__)) + '/ckpt/'


def main(argv=None):
    cnn = CNNModel()

    ckpt = tf.train.get_checkpoint_state(CKPT_PATH)
    if ckpt:
        cnn.saver.restore(cnn.sess, ckpt.model_checkpoint_path)
    else:
        print('ckpt is not exist.')
        exit(1)

    index = 100
    reader = RcImageReader(EVAL_FILE)
    record = reader.read(index)

    x = record.image_array.reshape([1, 60, 160, IMG_DIM])

    filter1 = cnn.sess.run(cnn.filter1,
                           feed_dict={cnn.input_holder: x,
                                      cnn.keepprob_holder: 1.0})

    print x.shape, filter1.shape

    # input画像のpng化(color)
    x = np.reshape(x, [60, 160, 2])
    dummy = np.zeros((60, 160, 1), np.uint8)
    image = np.dstack((x, dummy))
    imageshow = Image.fromarray(image.astype(np.uint8))
    with open('filter_img/input.png', 'wb') as out_png:
        imageshow.save(out_png, format='png')

    # filterのpng変化(mono)
    filter1 = np.reshape(filter1, [30, 80, 64])
    nml_m = 255 / filter1.max()
    filter1 = filter1 * nml_m
    # print nml_m
    f1_array = np.dsplit(filter1, 64)
    print f1_array[1].shape
    for i in range(len(f1_array)):
        f = np.reshape(f1_array[i], [30, 80])
        # print f
        imageshow = Image.fromarray(f.astype(np.uint8))
        filename = 'filter_img/filter1_%03d.png' % i
        with open(filename, 'wb') as out_png:
            imageshow.save(out_png, format='png')


if __name__ == '__main__':
    if not EVAL_FILE:
        print('EVAL_FILE is not exist')
        exit(1)

    tf.app.run()
