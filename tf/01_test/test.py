# -*- coding: utf-8 -*-
import tensorflow as tf
import multiprocessing as mp

core_num = mp.cpu_count()
config = tf.ConfigProto(
    inter_op_parallelism_threads=core_num,
    intra_op_parallelism_threads=core_num)
sess = tf.Session(config=config)

hello = tf.constant('hello, tensorflow!')
print sess.run(hello)
