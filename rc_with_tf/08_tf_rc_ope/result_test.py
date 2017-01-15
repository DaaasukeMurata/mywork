# coding: UTF-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time
import numpy as np

import tensorflow as tf
from tensorflow.python.framework import graph_util
from tensorflow.python.platform import gfile

import model
from reader import RcImageReader


# 推論したlogitsを表示する。訓練は無し

FLAGS = tf.app.flags.FLAGS
tf.app.flags.DEFINE_integer('epoch', 5, "訓練するEpoch数")
tf.app.flags.DEFINE_float('learning_rate', 0.001, "学習率")
tf.app.flags.DEFINE_string('data_dir', './data/', "訓練データのディレクトリ")
tf.app.flags.DEFINE_string('test_data', './data/eval.npy', "テストデータのパス")
tf.app.flags.DEFINE_string('checkpoint_dir', './checkpoints/', "チェックポイントを保存するディレクトリ")

filename = FLAGS.data_dir + 'train.npy'

# cross entropyを使った誤差関数


def _loss(logits, label):
    labels = tf.cast(label, tf.int64)
    cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(
        logits, labels, name='cross_entropy_per_example')
    cross_entropy_mean = tf.reduce_mean(cross_entropy, name='cross_entropy')
    return cross_entropy_mean


def _train(total_loss, global_step):
    opt = tf.train.AdamOptimizer(learning_rate=FLAGS.learning_rate)
    grads = opt.compute_gradients(total_loss)
    train_op = opt.apply_gradients(grads, global_step=global_step)
    return train_op


def main(argv=None):
    global_step = tf.Variable(0, trainable=False)

    # 入力データと、labelの入れ物を作る
    # shape=[height, width, depth]
    train_placeholder = tf.placeholder(tf.float32, shape=[60, 160, 1], name='input_image')
    label_placeholder = tf.placeholder(tf.int32, shape=[1], name='steer_label')
    keepprob_placeholder = tf.placeholder_with_default(tf.constant(1.0), shape=[], name='keep_prob')

    # (height, width, depth) -> (batch, height, width, depth)
    image_node = tf.expand_dims(train_placeholder, 0)

    logits = model.inference(image_node, keepprob_placeholder)
    total_loss = _loss(logits, label_placeholder)
    train_op = _train(total_loss, global_step)

    # evaluation用
    top_k_op = tf.nn.in_top_k(logits, label_placeholder, 1)

    saver = tf.train.Saver(tf.all_variables())

    with tf.Session() as sess:
        sess.run(tf.initialize_all_variables())
        _restore(saver, sess)

        reader = RcImageReader(FLAGS.test_data)
        true_count = 0
        for index in range(len(reader.bytes_array)):
            record = reader.read(index)

            predictions, logits_value = sess.run([top_k_op, logits],
                                                 feed_dict={train_placeholder: record.image_array,
                                                            label_placeholder: record.steer,
                                                            keepprob_placeholder: 1.0})

            answer = np.argmax(logits_value, 1)
            print('label %3d - answer %3d' % (record.steer, answer))


def _restore(saver, sess):
    checkpoint = tf.train.get_checkpoint_state(FLAGS.checkpoint_dir)
    if checkpoint and checkpoint.model_checkpoint_path:
        saver.restore(sess, checkpoint.model_checkpoint_path)


if __name__ == '__main__':
    tf.app.run()
