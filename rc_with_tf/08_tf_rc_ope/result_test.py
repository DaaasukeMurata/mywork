#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np

import tensorflow as tf

import model
from reader import RcImageReader


# define
EVAL_FILE = './data/eval.npy'
CHECKPOINT_PATH = os.path.abspath(os.path.dirname(__file__)) + '/checkpoints/'
KEEPPROB = 1.0


def main(argv=None):
    global_step = tf.Variable(0, trainable=False)

    # shape=[height, width, depth]
    input_holder = tf.placeholder(tf.float32, shape=[60, 160, 1], name='input_image')

    # (height, width, depth) -> (batch, height, width, depth)
    image_node = tf.expand_dims(input_holder, 0)
    logits = model.inference(image_node, KEEPPROB)
    saver = tf.train.Saver(tf.all_variables())

    with tf.Session() as sess:
        sess.run(tf.initialize_all_variables())
        _restore(saver, sess)

        reader = RcImageReader(EVAL_FILE)
        for index in range(len(reader.bytes_array)):
            record = reader.read(index)

            logits_val = sess.run(logits,
                                  feed_dict={input_holder: record.image_array})
            answer = np.argmax(logits_val, 1)
            print('label %3d - answer %3d' % (record.steer, answer))


def _restore(saver, sess):
    checkpoint = tf.train.get_checkpoint_state(CHECKPOINT_PATH)
    if checkpoint and checkpoint.model_checkpoint_path:
        saver.restore(sess, checkpoint.model_checkpoint_path)


if __name__ == '__main__':
    main()
