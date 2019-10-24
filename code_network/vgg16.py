#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu May 17 20:04:56 2018

@author: mengdan
"""

import tf_util
import tensorflow as tf

def vgg16(input, is_training, output_dim, bn_decay=None):
    """
    objective: build the network
    
    param: 
        input: image patch in brg (after pre-processing), [batch, height, width, 3], 
        is_training: training or testing
        bn_decay: None
        output_dim: final feature dimension
    """
    
    # block 1 (128x128x3 -> 64x64x64)
    conv1_1 = tf_util.conv2d(input, 64, [3,3], scope='conv1_1',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)  
    conv1_2 = tf_util.conv2d(conv1_1, 64, [3,3], scope='conv1_2',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)   
    pool1 = tf_util.max_pool2d(conv1_2, [2,2], scope='pool1',stride=[2,2],padding='VALID')
    
    # block 2 (64x64x64 -> 32x32x128)
    conv2_1 = tf_util.conv2d(pool1, 128, [3,3], scope='conv2_1',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)  
    conv2_2 = tf_util.conv2d(conv2_1, 128, [3,3], scope='conv2_2',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)   
    pool2 = tf_util.max_pool2d(conv2_2, [2,2], scope='pool2',stride=[2,2],padding='VALID')

    # block 3 (32x32x128 -> 16x16x256)
    conv3_1 = tf_util.conv2d(pool2, 256, [3,3], scope='conv3_1',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)  
    conv3_2 = tf_util.conv2d(conv3_1, 256, [3,3], scope='conv3_2',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)  
    conv3_3 = tf_util.conv2d(conv3_2, 256, [3,3], scope='conv3_3',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)   
    pool3 = tf_util.max_pool2d(conv3_3, [2,2], scope='pool3',stride=[2,2],padding='VALID')
    
    # block 4 (16x16x256 -> 16x16x512 -> 1x1x512)
    conv4_1 = tf_util.conv2d(pool3, 512, [3,3], scope='conv4_1',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)  
    conv4_2 = tf_util.conv2d(conv4_1, 512, [3,3], scope='conv4_2',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)  
    conv4_3 = tf_util.conv2d(conv4_2, 512, [3,3], scope='conv4_3',
                             stride=[1,1], padding='SAME', 
                             is_training=is_training, bn_decay=bn_decay)   
    # net = tf_util.max_pool2d(net, [2,2], scope='pool4',stride=[2,2],padding='VALID')
    pool4 = tf_util.avg_pool2d(conv4_3, [16,16], scope = 'pool4', stride=[16,16], padding='VALID')
    
    shape = pool4.get_shape().as_list()
    dim = 1
    for d in shape[1:]:
        dim *= d
    pool4 = tf.reshape(pool4, [-1, dim])
    print(pool4.shape)

    
    # block 5 (16x16x512 -> 1x512 -> 1x128)
    fc1 = tf_util.fully_connected(pool4, 512, scope='fc1', is_training = is_training)
#    fc1 = tf_util.dropout(fc1, keep_prob=0.7, is_training=is_training,
#                          scope='dp1')
    #fc2 = tf_util.fully_connected(fc1, output_dim, scope='fc2', is_training = is_training)
    fc2 = tf_util.fully_connected(fc1, output_dim, activation_fn=None, scope='fc2',is_training = is_training)
    
    out = tf.nn.l2_normalize(fc2, dim=1)   
    
    
    return out