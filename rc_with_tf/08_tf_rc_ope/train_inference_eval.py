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
tf.app.flags.DEFINE_string('data_dir', '../07_rosbag/d/', "訓練データのディレクトリ")
tf.app.flags.DEFINE_string('checkpoint_dir', './checkpoints/', "チェックポイントを保存するディレクトリ")
tf.app.flags.DEFINE_string('test_data', '../07_rosbag/d/003_learn_data.npy', "テストデータのパス")
tf.app.flags.DEFINE_string('graph_dir', './graphs/', "グラフを保存するディレクトリ")

filename = FLAGS.data_dir + '004_learn_data.npy'

# cross entropyを使った誤差関数


def _loss(logits, label):
    labels = tf.cast(label, tf.int64)
    cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(
        logits, labels, name='cross_entropy_per_example')
    cross_entropy_mean = tf.reduce_mean(cross_entropy, name='cross_entropy')
    return cross_entropy_mean


def _train(total_loss, global_step):
    # 勾配降下法
    opt = tf.train.GradientDescentOptimizer(learning_rate=FLAGS.learning_rate)
    grads = opt.compute_gradients(total_loss)
    train_op = opt.apply_gradients(grads, global_step=global_step)
    return train_op


def main(argv=None):
    global_step = tf.Variable(0, trainable=False)

    # 入力データと、labelの入れ物を作る
    # shape=[height, width, depth]
    train_placeholder = tf.placeholder(tf.float32, shape=[60, 160, 1], name='input_image')
    label_placeholder = tf.placeholder(tf.int32, shape=[1], name='steer_label')

    # (width, height, depth) -> (batch, width, height, depth)
    image_node = tf.expand_dims(train_placeholder, 0)

    logits = model.inference(image_node)
    total_loss = _loss(logits, label_placeholder)
    train_op = _train(total_loss, global_step)

    # evaluation用
    top_k_op = tf.nn.in_top_k(logits, label_placeholder, 1)

    saver = tf.train.Saver(tf.all_variables())

    with tf.Session() as sess:
        sess.run(tf.initialize_all_variables())

        total_duration = 0

        for epoch in range(1, FLAGS.epoch + 1):
            start_time = time.time()

            print('Epoch %d: %s' % (epoch, filename))
            reader = RcImageReader(filename)

            for index in range((epoch - 1) * 100, (epoch - 1) * 100 + 100):
                image = reader.read(index)

                _, loss_value, logits_value = sess.run([train_op, total_loss, logits],
                                                       feed_dict={train_placeholder: image.image_array,
                                                                  label_placeholder: image.steer_label})

                assert not np.isnan(loss_value), 'Model diverged with loss = NaN'

                if index % 100 == 0:
                    answer = np.argmax(logits_value, 1)
                    # print('[%d]: %d  logits_value:%f' % (image.steer_label, answer, logits_value[0][answer]))

            duration = time.time() - start_time
            total_duration += duration
            prediction = _eval(sess, top_k_op, train_placeholder, label_placeholder)

            print('epoch %d duration=%d sec, prediction=%.3f' % (epoch, duration, prediction))

            tf.train.SummaryWriter(FLAGS.checkpoint_dir, sess.graph)
            saver.save(sess, FLAGS.checkpoint_dir, global_step=epoch)
            # _export_graph(sess, epoch)

        print('Total duration = %d sec' % total_duration)


def _eval(sess, top_k_op, input_image, label_placeholder):
    if not FLAGS.test_data:
        return np.nan

    image_reader = RcImageReader(FLAGS.test_data)
    true_count = 0
    for index in range(300):
        image = image_reader.read(index)

        predictions = sess.run([top_k_op],
                               feed_dict={input_image: image.image_array,
                                          label_placeholder: image.steer_label})
        true_count += np.sum(predictions)

    return (true_count / 300.0)


def _restore(saver, sess):
    checkpoint = tf.train.get_checkpoint_state(FLAGS.checkpoint_dir)
    if checkpoint and checkpoint.model_checkpoint_path:
        saver.restore(sess, checkpoint.model_checkpoint_path)


def _export_graph(sess, epoch):
    constant_graph_def = graph_util.convert_variables_to_constants(sess, sess.graph_def, ["output/logits"])

    file_path = os.path.join(FLAGS.graph_dir, 'graph_%02d_epoch.pb' % epoch)
    with gfile.FastGFile(file_path, 'wb') as f:
        f.write(constant_graph_def.SerializeToString())


if __name__ == '__main__':
    tf.app.run()
