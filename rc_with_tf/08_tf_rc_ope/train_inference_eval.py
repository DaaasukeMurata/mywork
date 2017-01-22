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
tf.app.flags.DEFINE_integer('epoch', 10, "訓練するEpoch数")
tf.app.flags.DEFINE_float('learning_rate', 0.0001, "学習率")
tf.app.flags.DEFINE_string('data_dir', './data/', "訓練データのディレクトリ")
tf.app.flags.DEFINE_string('test_data', './data/eval.npy', "テストデータのパス")
tf.app.flags.DEFINE_string('checkpoint_dir', './checkpoints/', "チェックポイントを保存するディレクトリ")

filename = FLAGS.data_dir + 'train.npy'

BATCH_SIZE = 64


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
    train_placeholder = tf.placeholder(tf.float32, shape=[BATCH_SIZE, 60, 160, 1], name='input_image')
    label_placeholder = tf.placeholder(tf.int32, shape=[BATCH_SIZE], name='steer_label')
    keepprob_placeholder = tf.placeholder_with_default(tf.constant(1.0), shape=[], name='keep_prob')

    # (height, width, depth) -> (batch, height, width, depth)
    # image_node = tf.expand_dims(train_placeholder, 0)
    input_image = tf.placeholder_with_default(train_placeholder,
                                              [BATCH_SIZE, 60, 160, 1], name="input_image")

    logits = model.inference(input_image, keepprob_placeholder, BATCH_SIZE)
    total_loss = _loss(logits, label_placeholder)
    train_op = _train(total_loss, global_step)

    # evaluation用
    top_k_op = tf.nn.in_top_k(logits, label_placeholder, 1)

    saver = tf.train.Saver(tf.all_variables())

    batch_images = []
    batch_steers = []

    with tf.Session() as sess:
        sess.run(tf.initialize_all_variables())

        for epoch in range(1, FLAGS.epoch + 1):
            start_time = time.time()

            print('Epoch %d: %s' % (epoch, filename))
            reader = RcImageReader(filename)

            for index in range(len(reader.bytes_array)):
                record = reader.read(index)

                # mini batch用データ作成
                batch_images.append(record.image_array)
                batch_steers.append(record.steer[0])       # TODO steer[0]の [0]っている？
                if len(batch_images) < BATCH_SIZE:
                    continue

                _, loss_value, logits_value = sess.run([train_op, total_loss, logits],
                                                       feed_dict={
                                                           train_placeholder: batch_images,
                                                           label_placeholder: batch_steers,
                                                           keepprob_placeholder: 0.5})

                del batch_images[:]
                del batch_steers[:]

                print('epoch:%d index:%d , loss_value:%.3f'
                      % (epoch, index, loss_value))

            duration = time.time() - start_time

            prediction = _eval(sess, top_k_op, train_placeholder, label_placeholder)
            print('epoch %d duration=%d sec, prediction=%.3f' % (epoch, duration, prediction))

            tf.train.SummaryWriter(FLAGS.checkpoint_dir, sess.graph)
            saver.save(sess, FLAGS.checkpoint_dir, global_step=epoch)


def _eval(sess, top_k_op, input_image, label_placeholder):
    if not FLAGS.test_data:
        return np.nan

    reader = RcImageReader(FLAGS.test_data)
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
                               feed_dict={input_image: batch_images,
                                          label_placeholder: batch_steers})

        del batch_images[:]
        del batch_steers[:]

        true_count += np.sum(predictions)

    return (true_count / len(reader.bytes_array) - (len(reader.bytes_array) % BATCH_SIZE))


def _restore(saver, sess):
    checkpoint = tf.train.get_checkpoint_state(FLAGS.checkpoint_dir)
    if checkpoint and checkpoint.model_checkpoint_path:
        saver.restore(sess, checkpoint.model_checkpoint_path)


if __name__ == '__main__':
    tf.app.run()
