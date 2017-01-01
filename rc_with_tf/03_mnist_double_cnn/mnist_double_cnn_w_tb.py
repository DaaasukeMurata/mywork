
# coding: utf-8

# **[MDT-01]** 必要なモジュールをインポートして、乱数のシードを設定します。
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import os
from tensorflow.examples.tutorials.mnist import input_data

np.random.seed(20160704)
tf.set_random_seed(20160704)


# **[MDT-02]** MNISTのデータセットを用意します。

mnist = input_data.read_data_sets("/tmp/data/", one_hot=True)


# **[MDT-03]** 畳込みフィルターが1層のCNNを表現するクラスを定義します。

class DoubleCNN:

    def __init__(self, num_filters1, num_filters2, num_units, keep_prob):
        with tf.Graph().as_default():
            self.prepare_model(num_filters1, num_filters2,
                               num_units, keep_prob)
            self.prepare_session()

    def prepare_model(self, num_filters1, num_filters2, num_units, keep_prob):
        num_units1 = 7 * 7 * num_filters2
        num_units2 = num_units

        with tf.name_scope('input'):
            x = tf.placeholder(tf.float32, [None, 784], name='input')
            x_image = tf.reshape(x, [-1, 28, 28, 1])

        with tf.name_scope('convolution1'):
            W_conv1 = tf.Variable(tf.truncated_normal([5, 5, 1, num_filters1], stddev=0.1), name='conv-filter1')
            h_conv1 = tf.nn.conv2d(x_image, W_conv1, strides=[1, 1, 1, 1], padding='SAME', name='filter-output1')

        with tf.name_scope('pooling1'):
            b_conv1 = tf.Variable(tf.constant(0.1, shape=[num_filters1]), name='relu-filter1')
            h_conv1_cutoff = tf.nn.relu(h_conv1 + b_conv1)
            h_pool1 = tf.nn.max_pool(h_conv1_cutoff, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME', name='max-pool1')

        with tf.name_scope('convolution2'):
            W_conv2 = tf.Variable(tf.truncated_normal([5, 5, num_filters1, num_filters2], stddev=0.1), name='conv-filter2')
            h_conv2 = tf.nn.conv2d(h_pool1, W_conv2, strides=[1, 1, 1, 1], padding='SAME', name='filter-output2')

        with tf.name_scope('pooling2'):
            b_conv2 = tf.Variable(tf.constant(0.1, shape=[num_filters2]), name='relu-filter2')
            h_conv2_cutoff = tf.nn.relu(h_conv2 + b_conv2)
            h_pool2 = tf.nn.max_pool(h_conv2_cutoff, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME', name='max-pool2')
            h_pool2_flat = tf.reshape(h_pool2, [-1, 7 * 7 * num_filters2], name='pool-output2')

        with tf.name_scope('fully-connected'):
            w2 = tf.Variable(tf.truncated_normal([num_units1, num_units2]))
            b2 = tf.Variable(tf.zeros([num_units2]))
            hidden2 = tf.nn.relu(tf.matmul(h_pool2_flat, w2) + b2, name='fc-output')

        with tf.name_scope('dropout'):
            hidden2_drop = tf.nn.dropout(hidden2, keep_prob, name='dropout-output')

        with tf.name_scope('softmax'):
            # hidden2_drop = tf.nn.dropout(hidden2, keep_prob)

            w0 = tf.Variable(tf.zeros([num_units2, 10]))
            b0 = tf.Variable(tf.zeros([10]))
            p = tf.nn.softmax(tf.matmul(hidden2_drop, w0) + b0, name='softmax-output')

        with tf.name_scope('optimizer'):
            t = tf.placeholder(tf.float32, [None, 10], name='labels')
            loss = -tf.reduce_sum(t * tf.log(p), name='loss')
            train_step = tf.train.AdamOptimizer(0.0001).minimize(loss)

        with tf.name_scope('evaluator'):
            correct_prediction = tf.equal(tf.argmax(p, 1), tf.argmax(t, 1))
            accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32), name='accuracy')

        tf.scalar_summary("loss", loss)
        tf.scalar_summary("accuracy", accuracy)
        tf.histogram_summary("convolution_filters1", W_conv1)
        tf.histogram_summary("convolution_filters2", W_conv2)

        self.x, self.t, self.p = x, t, p
        self.train_step = train_step
        self.loss = loss
        self.accuracy = accuracy

    def prepare_session(self):
        sess = tf.InteractiveSession()
        sess.run(tf.initialize_all_variables())
        summary = tf.merge_all_summaries()
        writer = tf.train.SummaryWriter("/tmp/tensorboard", sess.graph)

        self.sess = sess
        self.summary = summary
        self.writer = writer


# **[MDT-04]** TensorBoard用のデータ出力ディレクトリーを削除して初期化しておきます。
os.system(u'rm -rf /tmp/tensorboard')


# **[MDT-05]** パラメーターの最適化を4000回繰り返します。テストセットに対して約98%の正解率が得られます。
cnn = DoubleCNN(4, 8, 1024, 0.5)

i = 0
for _ in range(2000):
    i += 1
    batch_xs, batch_ts = mnist.train.next_batch(50)
    cnn.sess.run(cnn.train_step,
                 feed_dict={cnn.x: batch_xs, cnn.t: batch_ts})
    if i % 50 == 0:
        summary, loss_val, acc_val = cnn.sess.run(
            [cnn.summary, cnn.loss, cnn.accuracy],
            feed_dict={cnn.x: mnist.test.images, cnn.t: mnist.test.labels})
        print ('Step: %d, Loss: %f, Accuracy: %f' % (i, loss_val, acc_val))
        cnn.writer.add_summary(summary, i)
