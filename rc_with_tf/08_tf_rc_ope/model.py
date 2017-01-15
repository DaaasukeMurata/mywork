# coding: UTF-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

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
