#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun May 27 10:48:20 2018

@author: mengdan
"""

import tf_util
    
def metricNet(input, is_training, output_dim, bn_decay=None):
    """
    objective: build the network
    
    param: 
        input: Bx256, concat of img_feature+pos_feature or img_feature+neg_feature
        is_training: training or testing
        bn_decay: None
        output_dim: Bx2, final feature dimension
    """
    
#    # concat along row, Bx256
#    img_pos_feature = tf.concat([img_batch_feature, pos_pcl_feature], 1)      
#    img_neg_feature = tf.concat([img_batch_feature, neg_pcl_feature], 1)    # concat along row
    
    fc1 = tf_util.fully_connected(input, 128, scope='fc1', is_training = is_training)
    fc2 = tf_util.fully_connected(fc1, 64, scope='fc2', is_training = is_training)
    fc3 = tf_util.fully_connected(fc2, output_dim, activation_fn=None, scope='fc3',is_training = is_training)
    
    return fc3


    
    
    
    