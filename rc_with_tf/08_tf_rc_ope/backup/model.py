#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np
import tensorflow as tf


# 最終出力の数
NUM_CLASSES = 180


def _get_weights(shape, stddev=1.0):
    var = tf.get_variable('weights', shape,
                          initializer=tf.truncated_normal_initializer(stddev=stddev))
    return var


def _get_biases(shape, value=0.0):
    var = tf.get_variable('biases', shape,
                          initializer=tf.constant_initializer(value))
    return var


def inference(image_node, keep_prob, batch_size=1):
    # conv1
    with tf.variable_scope('conv1') as scope:
        weights = _get_weights(shape=[5, 5, 1, 64], stddev=1e-4)
        conv = tf.nn.conv2d(image_node, weights, [1, 1, 1, 1], padding='SAME')
        biases = _get_biases([64], value=0.1)
        bias = tf.nn.bias_add(conv, biases)
        conv1 = tf.nn.relu(bias, name=scope.name)

    # pool1
    pool1 = tf.nn.max_pool(conv1, ksize=[1, 3, 3, 1], strides=[1, 2, 2, 1],
                           padding='SAME', name='pool1')

    # conv2
    with tf.variable_scope('conv2') as scope:
        weights = _get_weights(shape=[5, 5, 64, 64], stddev=1e-2)
        conv = tf.nn.conv2d(pool1, weights, [1, 1, 1, 1], padding='SAME')
        biases = _get_biases([64], value=0.1)
        bias = tf.nn.bias_add(conv, biases)
        conv2 = tf.nn.relu(bias, name=scope.name)

    # pool2
    pool2 = tf.nn.max_pool(conv2, ksize=[1, 3, 3, 1], strides=[1, 2, 2, 1],
                           padding='SAME', name='pool2')

    reshape = tf.reshape(pool2, [batch_size, -1])
    dim = reshape.get_shape()[1].value

    # fc3
    with tf.variable_scope('fc3') as scope:
        weights = _get_weights(shape=[dim, 384], stddev=0.04)
        biases = _get_biases([384], value=0.1)
        fc3 = tf.nn.dropout(reshape, keep_prob)
        fc3 = tf.nn.relu(tf.matmul(reshape, weights) + biases, name=scope.name)

    # fc4
    with tf.variable_scope('fc4') as scope:
        weights = _get_weights(shape=[384, 192], stddev=0.04)
        biases = _get_biases([192], value=0.1)
        fc3 = tf.nn.dropout(fc3, keep_prob)
        fc4 = tf.nn.relu(tf.matmul(fc3, weights) + biases, name=scope.name)

    # output
    with tf.variable_scope('output') as scope:
        weights = _get_weights(shape=[192, NUM_CLASSES], stddev=1 / 192.0)
        biases = _get_biases([NUM_CLASSES], value=0.0)
        logits = tf.add(tf.matmul(fc4, weights), biases, name='logits')

    return logits


class CNNModel():
    NUM_FILTER1 = 64
    NUM_FILTER2 = 64
    NUM_CLASSES = 180
    SUMMARY_PATH = os.path.abspath(os.path.dirname(__file__)) + '/tensorboard/'

    def __init__(self, keep_prob, batch_size=1):  # TODO batchsizeに依存しないようにする
        with tf.Graph().as_default():
            self.prepare_model(keep_prob, batch_size)
            self.prepare_session()

    def prepare_model(keep_prob, batch_size=1):  # TODO batchsizeに依存しないようにする

        with tf.name_scope('input'):
            # TODO batchsizeに依存しないようにする
            input_holder = tf.placeholder(tf.float32, shape=[batch_size, 60, 160, 1], name='input_image')

        with tf.name_scope('conv1'):
            W_conv1 = tf.Variable(tf.truncated_normal([5, 5, 1, NUM_FILTER1], stddev=1e-2), name='conv-filter1')
            h_conv1_wk = tf.nn.conv2d(input_holder, W_conv1, strides=[1, 1, 1, 1],
                                      padding='SAME', name='filter-output1')
            b_conv1 = tf.Variable(tf.constant(0.1, shape=[NUM_FILTER1]), name='relu-filter1')
            h_conv1 = tf.nn.relu(h_conv1_wk + b_conv1)

        with tf.name_scope('pool1'):
            h_pool1 = tf.nn.max_pool(h_conv1, ksize=[1, 3, 3, 1], strides=[1, 2, 2, 1],
                                     padding='SAME', name='pool1-output')

        with tf.name_scope('conv2'):
            W_conv2 = tf.Variable(tf.truncated_normal([5, 5, NUM_FILTER1, NUM_FILTER2], stddev=1e-2),
                                  name='conv-filter2')
            h_conv2_wk = tf.nn.conv2d(h_pool1, W_conv2, strides=[1, 1, 1, 1], padding='SAME', name='filter-output2')
            b_conv2 = tf.Variable(tf.constant(0.1, shape=[NUM_FILTER2]), name='relu-filter2')
            h_conv2 = tf.nn.relu(h_conv2_wk + b_conv2)

        with tf.name_scope('pool2'):
            h_pool2 = tf.nn.max_pool(h_conv2, ksize=[1, 3, 3, 1], strides=[1, 2, 2, 1], padding='SAME', name='pool2')

            h_pool2_flat = tf.reshape(h_pool2, [batch_size, -1], name='pool2-output')　  # TODO batchsizeに依存しないようにする

        with tf.name_scope('fc1'):
            dim = hpool2_flat.get_shape()[1].value
            w2 = tf.Variable(tf.truncated_normal([dim, 384]))  # TODO stddevなくして精度落ちないか？
            b2 = tf.Variable(tf.zeros([384]))  # TODO stddevなくして精度落ちないか？
            h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, w2) + b2, name='fc1-output')
            h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

        with tf.name_scope('fc2'):
            w2 = tf.Variable(tf.truncated_normal([384, 192]))  # TODO stddevなくして精度落ちないか？
            b2 = tf.Variable(tf.zeros([192]))  # TODO stddevなくして精度落ちないか？
            h_fc2 = tf.nn.relu(tf.matmul(h_fc1_drop, w2) + b2, name='fc2-output')

        with tf.name_scope('softmax'):
            w0 = tf.Variable(tf.zeros([192, NUM_OUTPUT]))
            b0 = tf.Variable(tf.zeros([NUM_OUTPUT]))
            predictions = tf.nn.softmax(tf.matmul(h_fc2, w0) + b0, name='softmax-output')

        with tf.name_scope('optimizer'):
            label_holder = tf.placeholder(tf.float32, [batch_size], name='labels')
            loss = -tf.reduce_sum(label_holder * tf.log(predictions), name='loss')
            train_step = tf.train.AdamOptimizer(0.0001).minimize(loss)

        with tf.name_scope('evaluator'):
            correct_prediction = tf.equal(tf.argmax(predictions, 1), tf.argmax(label_holder, 1))
            accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32), name='accuracy')

        tf.scalar_summary("loss", loss)
        tf.scalar_summary("accuracy", accuracy)
        tf.histogram_summary("conv_filter1", W_conv1)
        tf.histogram_summary("conv_filter2", W_conv2)

        self.input_holder = input_holder
        self.label_holder = label_holder
        self.predictions = predictions
        self.train_step = train_step
        self.loss = loss
        self.accuracy = accuracy

    def prepare_session(self):
        sess = tf.InteractiveSession()
        sess.run(tf.initialize_all_variables())
        summary = tf.merge_all_summaries()
        writer = tf.train.SummaryWriter(SUMMARY_PATH, sess.graph)

        self.sess = sess
        self.summary = summary
        self.writer = writer
