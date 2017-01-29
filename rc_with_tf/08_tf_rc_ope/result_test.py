#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np

import tensorflow as tf

from model import CNNModel
from reader import RcImageReader


# define
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

    reader = RcImageReader(EVAL_FILE)
    correct_count = 0
    for index in range(len(reader.bytes_array)):
        record = reader.read(index)

        x = record.image_array.reshape([1, 60, 160, 1])
        t = record.steer_array.reshape([1, 180])

        p, acc = cnn.sess.run([cnn.predictions, cnn.accuracy],
                              feed_dict={cnn.input_holder: x,
                                         cnn.label_holder: t,
                                         cnn.keepprob_holder: 1.0})

        correct_count += acc
        label = np.argmax(record.steer_array)
        answer = np.argmax(p, 1)
        print('label%3d - answer%3d' % (label, answer))

    print('total accuracy : %f' % (float(correct_count) / index))


if __name__ == '__main__':
    if not EVAL_FILE:
        print('EVAL_FILE is not exist')
        exit(1)

    tf.app.run()
