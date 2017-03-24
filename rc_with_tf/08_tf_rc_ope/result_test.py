#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np

import tensorflow as tf

from model import CNNModel
from reader import RcImageReader


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
    # cnn.saver.restore(cnn.sess, CKPT_PATH + 'model-1071')

    reader = RcImageReader(EVAL_FILE)
    correct_count = 0
    for index in range(len(reader.bytes_array)):
        record = reader.read(index)

        img = record.image_array.reshape([1, 60, 160, IMG_DIM])
        line_list = [record.f_line.x1, record.f_line.y1, record.f_line.x2, record.f_line.y2, record.f_line.piangle,
                     record.l_line.x1, record.l_line.y1, record.l_line.x2, record.l_line.y2, record.l_line.piangle]
        line = np.array(line_list)
        line = line.reshape([1, 10])
        t = record.steer_array.reshape([1, 180])

        p, acc = cnn.sess.run([cnn.predictions, cnn.accuracy],
                              feed_dict={cnn.image_holder: img,
                                         cnn.line_meta_holder: line,
                                         cnn.label_holder: t,
                                         cnn.keepprob_holder: 1.0})

        correct_count += acc
        label = np.argmax(record.steer_array)
        answer = np.argmax(p, 1)

        print('label%3d-answer%3d ' % (label, answer)),
        print('f_line %3d, %3d, %3d, %3d, %.2f ' % (record.f_line.x1, record.f_line.y1, record.f_line.x2, record.f_line.y2, record.f_line.piangle)),
        print('l_line %3d, %3d, %3d, %3d, %.2f ' % (record.l_line.x1, record.l_line.y1, record.l_line.x2, record.l_line.y2, record.l_line.piangle)),
        print

    print('total accuracy : %f' % (float(correct_count) / index))


if __name__ == '__main__':
    if not EVAL_FILE:
        print('EVAL_FILE is not exist')
        exit(1)

    tf.app.run()
