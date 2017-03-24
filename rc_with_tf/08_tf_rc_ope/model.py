#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np
import tensorflow as tf

IMG_DIM = 2
NUM_FILTER1 = 64
NUM_FILTER2 = 64
NUM_HIDDEN1 = 64
NUM_HIDDEN2 = 64
NUM_OUTPUT = 180
SUMMARY_PATH = os.path.abspath(os.path.dirname(__file__)) + '/tensorboard/'


class CNNModel():

    def __init__(self):
        with tf.Graph().as_default():
            self.prepare_model()
            self.prepare_session()

    def prepare_model(self):

        with tf.name_scope('input'):
            image_holder = tf.placeholder(tf.float32, shape=[None, 60, 160, IMG_DIM], name='input_image')

        with tf.name_scope('conv1'):
            CH_MULTI = 32
            Dep_conv1 = tf.Variable(tf.truncated_normal([5, 5, IMG_DIM, CH_MULTI], stddev=1e-2),
                                    name='conv1-depthwise-filter')
            Poi_conv1 = tf.Variable(tf.truncated_normal([1, 1, IMG_DIM * CH_MULTI, NUM_FILTER1], stddev=1e-2),
                                    name='conv1-pointwise-filter')
            h_conv1_wk = tf.nn.separable_conv2d(image_holder, Dep_conv1, Poi_conv1, strides=[1, 1, 1, 1],
                                                padding='SAME', name='conv2-output')
            b_conv1 = tf.Variable(tf.constant(0.1, shape=[NUM_FILTER1]), name='relu-filter1')
            h_conv1 = tf.nn.relu(h_conv1_wk + b_conv1)

        with tf.name_scope('pool1'):
            h_pool1 = tf.nn.max_pool(h_conv1, ksize=[1, 3, 3, 1], strides=[1, 2, 2, 1],
                                     padding='SAME', name='pool1-output')

        with tf.name_scope('conv2'):
            W_conv2 = tf.Variable(tf.truncated_normal([5, 5, NUM_FILTER1, NUM_FILTER2], stddev=1e-2),
                                  name='conv2-filter')
            h_conv2_wk = tf.nn.conv2d(h_pool1, W_conv2, strides=[1, 1, 1, 1], padding='SAME', name='conv2-output')
            b_conv2 = tf.Variable(tf.constant(0.1, shape=[NUM_FILTER2]), name='relu2-filter')
            h_conv2 = tf.nn.relu(h_conv2_wk + b_conv2)

        with tf.name_scope('pool2'):
            h_pool2 = tf.nn.max_pool(h_conv2, ksize=[1, 3, 3, 1], strides=[1, 2, 2, 1], padding='SAME', name='pool2')
            h_pool2_flat = tf.reshape(h_pool2, [-1, (15 * 40 * 1 * NUM_FILTER2)], name='pool2-output')

        with tf.name_scope('fc1'):
            dim = h_pool2_flat.get_shape()[1].value
            w1 = tf.Variable(tf.truncated_normal([dim, 384]))
            b1 = tf.Variable(tf.zeros([384]))
            h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, w1) + b1, name='fc1-output')
            h_fc1_flat = tf.reshape(h_fc1, [-1, 384], name='fc1-flat')
            # print h_fc1_flat

        with tf.name_scope('input_line_meta'):
            line_meta_holder = tf.placeholder(tf.float32, shape=[None, 10], name='input_line_meta')
            # print line_meta_holder

        with tf.name_scope('hidden1'):
            w_hidden1 = tf.Variable(tf.truncated_normal([10, NUM_HIDDEN1]), name='hidden1-W')
            b_hidden1 = tf.Variable(tf.zeros([NUM_HIDDEN1]), name='hidden1-b')
            h_hidden1 = tf.nn.relu(tf.matmul(line_meta_holder, w_hidden1) + b_hidden1, name='hidden1-output')
            # print h_hidden1

        with tf.name_scope('hidden2'):
            w_hidden2 = tf.Variable(tf.truncated_normal([NUM_HIDDEN1, NUM_HIDDEN2]), name='hidden2-W')
            b_hidden2 = tf.Variable(tf.zeros([NUM_HIDDEN2]), name='hidden2-b')
            h_hidden2 = tf.nn.relu(tf.matmul(h_hidden1, w_hidden2) + b_hidden2, name='hidden2-output')
            h_hidden2_flat = tf.reshape(h_hidden2, [-1, NUM_HIDDEN2], name='hidden2-flat')
            # print h_hidden2
            # print h_hidden2_flat

        with tf.name_scope('fc2'):
            # numpy hstack
            fc_array = tf.concat(1, [h_fc1_flat, h_hidden2_flat])
            dim = fc_array.get_shape()[1].value
            w2 = tf.Variable(tf.truncated_normal([dim, 192]))
            b2 = tf.Variable(tf.zeros([192]))
            h_fc2 = tf.nn.relu(tf.matmul(fc_array, w2) + b2, name='fc2-output')
            keepprob_holder = tf.placeholder_with_default(tf.constant(1.0), shape=[], name='keep_prob')
            h_fc2_drop = tf.nn.dropout(h_fc2, keepprob_holder)

        with tf.name_scope('fc3'):
            w3 = tf.Variable(tf.truncated_normal([192, 192]))
            b3 = tf.Variable(tf.zeros([192]))
            h_fc3 = tf.nn.relu(tf.matmul(h_fc2_drop, w3) + b3, name='fc3-output')

        with tf.name_scope('softmax'):
            w0 = tf.Variable(tf.zeros([192, NUM_OUTPUT]))
            b0 = tf.Variable(tf.zeros([NUM_OUTPUT]))
            predictions = tf.nn.softmax(tf.matmul(h_fc3, w0) + b0, name='softmax-output')

        with tf.name_scope('optimizer'):
            label_holder = tf.placeholder(tf.float32, [None, NUM_OUTPUT], name='labels')
            loss = -tf.reduce_sum(label_holder * tf.log(tf.clip_by_value(predictions, 1e-10, 1.0)), name='loss')
            train_step = tf.train.AdamOptimizer(0.0001).minimize(loss)

        with tf.name_scope('evaluator'):
            correct_prediction = tf.equal(tf.argmax(predictions, 1), tf.argmax(label_holder, 1))
            accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32), name='accuracy')

        tf.scalar_summary("loss", loss)
        tf.scalar_summary("accuracy", accuracy)
        tf.histogram_summary("conv1_depthwise_filter", Dep_conv1)
        tf.histogram_summary("conv1_pointwise_filter", Poi_conv1)
        tf.histogram_summary("conv2_filter", W_conv2)
        tf.histogram_summary("hidden1-W", w_hidden1)
        tf.histogram_summary("hidden2-W", w_hidden2)

        self.image_holder = image_holder
        self.line_meta_holder = line_meta_holder
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
