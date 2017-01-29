#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np
import tensorflow as tf

NUM_FILTER1 = 64
NUM_FILTER2 = 64
NUM_OUTPUT = 180
SUMMARY_PATH = os.path.abspath(os.path.dirname(__file__)) + '/tensorboard/'


class CNNModel():

    def __init__(self):
        with tf.Graph().as_default():
            self.prepare_model()
            self.prepare_session()

    def prepare_model(self):

        with tf.name_scope('input'):
            input_holder = tf.placeholder(tf.float32, shape=[None, 60, 160, 1], name='input_image')

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
            h_pool2_flat = tf.reshape(h_pool2, [-1, (15 * 40 * 1 * NUM_FILTER2)], name='pool2-output')

        # TODO stddevなくして精度落ちないか？
        with tf.name_scope('fc1'):
            dim = h_pool2_flat.get_shape()[1].value
            w2 = tf.Variable(tf.truncated_normal([dim, 384]))
            b2 = tf.Variable(tf.zeros([384]))
            h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, w2) + b2, name='fc1-output')
            keepprob_holder = tf.placeholder_with_default(tf.constant(1.0), shape=[], name='keep_prob')
            h_fc1_drop = tf.nn.dropout(h_fc1, keepprob_holder)

        with tf.name_scope('fc2'):
            w2 = tf.Variable(tf.truncated_normal([384, 192]))
            b2 = tf.Variable(tf.zeros([192]))
            h_fc2 = tf.nn.relu(tf.matmul(h_fc1_drop, w2) + b2, name='fc2-output')

        with tf.name_scope('softmax'):
            w0 = tf.Variable(tf.zeros([192, NUM_OUTPUT]))
            b0 = tf.Variable(tf.zeros([NUM_OUTPUT]))
            predictions = tf.nn.softmax(tf.matmul(h_fc2, w0) + b0, name='softmax-output')

        with tf.name_scope('optimizer'):
            label_holder = tf.placeholder(tf.float32, [None, NUM_OUTPUT], name='labels')
            loss = -tf.reduce_sum(label_holder * tf.log(predictions), name='loss')
            train_step = tf.train.AdamOptimizer(0.0001).minimize(loss)

# def _loss(logits, label):
#     labels = tf.cast(label, tf.int64)
#     cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(
#         logits, labels, name='cross_entropy_per_example')
#     cross_entropy_mean = tf.reduce_mean(cross_entropy, name='cross_entropy')
#     return cross_entropy_mean

# def _train(total_loss, global_step):
#     opt = tf.train.AdamOptimizer(learning_rate=LEARNING_RATE)
#     grads = opt.compute_gradients(total_loss)
#     train_op = opt.apply_gradients(grads, global_step=global_step)
#     return train_op

        with tf.name_scope('evaluator'):
            correct_prediction = tf.equal(tf.argmax(predictions, 1), tf.argmax(label_holder, 1))
            accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32), name='accuracy')

        tf.scalar_summary("loss", loss)
        tf.scalar_summary("accuracy", accuracy)
        tf.histogram_summary("conv_filter1", W_conv1)
        tf.histogram_summary("conv_filter2", W_conv2)

        self.input_holder = input_holder
        self.label_holder = label_holder
        self.keepprob_holder = keepprob_holder
        self.predictions = predictions
        self.train_step = train_step
        self.loss = loss
        self.accuracy = accuracy

    def prepare_session(self):
        sess = tf.InteractiveSession()
        sess.run(tf.initialize_all_variables())
        summary = tf.merge_all_summaries()
        writer = tf.train.SummaryWriter(SUMMARY_PATH, sess.graph)
        saver = tf.train.Saver()

        self.sess = sess
        self.summary = summary
        self.writer = writer
        self.saver = saver
