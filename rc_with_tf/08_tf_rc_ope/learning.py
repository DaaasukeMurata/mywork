#!/usr/bin/env python
# coding: UTF-8

import os
import time
import numpy as np

import tensorflow as tf

from model import CNNModel
from reader import RcImageReader


# define
EPOCH_NUM = 5      # 訓練する数
BATCH_SIZE = 64     # バッチサイズ
KEEP_PROB = 0.5
TRAIN_FILE = os.path.abspath(os.path.dirname(__file__)) + '/data/train.npy'
EVAL_FILE = os.path.abspath(os.path.dirname(__file__)) + '/data/eval.npy'
SUMMARY_PATH = os.path.abspath(os.path.dirname(__file__)) + '/tensorboard/'
CKPT_PATH = os.path.abspath(os.path.dirname(__file__)) + '/ckpt/'


def main(argv=None):
    os.system('rm -rf ' + SUMMARY_PATH)

    cnn = CNNModel()

    step = 0
    for epoch in range(1, EPOCH_NUM + 1):
        start_time = time.time()

        print('Epoch %d: %s' % (epoch, TRAIN_FILE))
        reader = RcImageReader(TRAIN_FILE)
        reader.shuffle()

        batch_images = []
        batch_lines = []
        batch_steers = []
        for index in range(len(reader.bytes_array)):
            record = reader.read(index)

            # mini batch用データ作成
            batch_images.append(record.image_array)
            batch_lines.append([record.f_line.x1, record.f_line.y1,
                                record.f_line.x2, record.f_line.y2,
                                record.f_line.piangle,
                                record.l_line.x1, record.l_line.y1,
                                record.l_line.x2, record.l_line.y2,
                                record.l_line.piangle])
            batch_steers.append(record.steer_array)
            if len(batch_images) < BATCH_SIZE:
                continue

            cnn.sess.run(cnn.train_step,
                         feed_dict={cnn.image_holder: batch_images,
                                    cnn.line_meta_holder: batch_lines,
                                    cnn.label_holder: batch_steers,
                                    cnn.keepprob_holder: KEEP_PROB})

            step += 1
            _eval(cnn, step)

            del batch_images[:]
            del batch_lines[:]
            del batch_steers[:]

        cnn.saver.save(cnn.sess, CKPT_PATH + 'model', global_step=step)
        duration = time.time() - start_time
        print('duration %d' % (duration))


def _eval(cnn, step):
    reader = RcImageReader(EVAL_FILE)
    reader.shuffle()

    all_images = []
    all_lines = []
    all_steers = []

    for index in range(100):
        record = reader.read(index)
        all_images.append(record.image_array)
        all_lines.append([record.f_line.x1, record.f_line.y1,
                          record.f_line.x2, record.f_line.y2,
                          record.f_line.piangle,
                          record.l_line.x1, record.l_line.y1,
                          record.l_line.x2, record.l_line.y2,
                          record.l_line.piangle])
        all_steers.append(record.steer_array)

    summary, loss_val, acc_val = cnn.sess.run([cnn.summary, cnn.loss, cnn.accuracy],
                                              feed_dict={cnn.image_holder: all_images,
                                                         cnn.line_meta_holder: all_lines,
                                                         cnn.label_holder: all_steers,
                                                         cnn.keepprob_holder: 1.0})

    print ('Step:%3d, Loss: %f, Accuracy: %f' % (step, loss_val, acc_val))
    cnn.writer.add_summary(summary, step)

    return


if __name__ == '__main__':
    if not EVAL_FILE:
        print('EVAL_FILE is not exist')
        exit(1)
    if not TRAIN_FILE:
        print('TRAIN_FILE is not exist')
        exit(1)

    tf.app.run()
