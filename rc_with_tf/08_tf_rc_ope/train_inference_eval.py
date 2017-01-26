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


#define
LEARNING_RATE = 0.0001  # 学習率
EPOCH_NUM = 10      # 訓練する数
BATCH_SIZE = 64     # バッチサイズ
TRAIN_FILE = './data/train.npy'
EVAL_FILE = './data/eval.npy'
CHECKPOINT_PATH = './checkpoints/'


def _loss(logits, label):
    labels = tf.cast(label, tf.int64)
    cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(
        logits, labels, name='cross_entropy_per_example')
    cross_entropy_mean = tf.reduce_mean(cross_entropy, name='cross_entropy')
    return cross_entropy_mean


def _train(total_loss, global_step):
    opt = tf.train.AdamOptimizer(learning_rate=LEARNING_RATE)
    grads = opt.compute_gradients(total_loss)
    train_op = opt.apply_gradients(grads, global_step=global_step)
    return train_op


def main(argv=None):
    global_step = tf.Variable(0, trainable=False)

    # 入力データと、labelの入れ物を作る
    # shape=[batch, height, width, depth]
    input_holder = tf.placeholder(tf.float32, shape=[BATCH_SIZE, 60, 160, 1], name='input_image')
    label_holder = tf.placeholder(tf.int32, shape=[BATCH_SIZE], name='steer_label')
    keepprob_holder = tf.placeholder_with_default(tf.constant(1.0), shape=[], name='keep_prob')

    logits = model.inference(input_holder, keepprob_holder, BATCH_SIZE)
    total_loss = _loss(logits, label_holder)
    train_op = _train(total_loss, global_step)
    top_k_op = tf.nn.in_top_k(logits, label_holder, 1)

    saver = tf.train.Saver(tf.all_variables())

    batch_images = []
    batch_steers = []

    with tf.Session() as sess:
        sess.run(tf.initialize_all_variables())

        for epoch in range(1, EPOCH_NUM + 1):
            start_time = time.time()

            print('Epoch %d: %s' % (epoch, TRAIN_FILE))
            reader = RcImageReader(TRAIN_FILE)

            for index in range(len(reader.bytes_array)):
                record = reader.read(index)

                # mini batch用データ作成
                batch_images.append(record.image_array)
                batch_steers.append(record.steer[0])       # TODO steer[0]の [0]っている？
                if len(batch_images) < BATCH_SIZE:
                    continue

                _, loss_value, logits_value = sess.run([train_op, total_loss, logits],
                                                       feed_dict={
                                                           input_holder: batch_images,
                                                           label_holder: batch_steers,
                                                           keepprob_holder: 0.5})

                del batch_images[:]
                del batch_steers[:]

                print('epoch:%d index:%d , loss_value:%.3f'
                      % (epoch, index, loss_value))

            duration = time.time() - start_time

            prediction = _eval(sess, top_k_op, input_holder, label_holder)
            print('epoch %d duration=%d sec, prediction=%.3f' % (epoch, duration, prediction))

            tf.train.SummaryWriter(CHECKPOINT_PATH, sess.graph)
            saver.save(sess, CHECKPOINT_PATH, global_step=epoch)


def _eval(sess, top_k_op, input_holder, label_holder):
    if not EVAL_FILE:
        return np.nan

    reader = RcImageReader(EVAL_FILE)
    true_count = 0
    batch_images = []
    batch_steers = []
    for index in range(len(reader.bytes_array)):
        record = reader.read(index)

        batch_images.append(record.image_array)
        batch_steers.append(record.steer[0])
        if len(batch_images) < BATCH_SIZE:
            continue

        predictions = sess.run([top_k_op],
                               feed_dict={input_holder: batch_images,
                                          label_holder: batch_steers})

        del batch_images[:]
        del batch_steers[:]

        true_count += np.sum(predictions)

    return (true_count / len(reader.bytes_array) - (len(reader.bytes_array) % BATCH_SIZE))


def _restore(saver, sess):
    checkpoint = tf.train.get_checkpoint_state(CHECKPOINT_PATH)
    if checkpoint and checkpoint.model_checkpoint_path:
        saver.restore(sess, checkpoint.model_checkpoint_path)


if __name__ == '__main__':
    tf.app.run()
