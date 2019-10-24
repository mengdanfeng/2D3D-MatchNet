#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 14 00:40:27 2018

@author: mengdan
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 21:35:04 2018

@author: mengdan
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jun  1 21:17:55 2018

@author: mengdan
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jun  1 10:43:52 2018

@author: mengdan
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May 18 20:25:02 2018

@author: mengdan
"""

import tensorflow as tf
import numpy as np

from vgg16 import vgg16
from pointNet import pointNet
from read_dataset import Data, splitTrainTest, shuffleTrainset

import os
import cv2

batch_size = 16
image_size = 128
pcl_size = 1024
image_feature_dim = 64
pcl_feature_dim = 64

thresh_dist = 0.5

date_time = '2015-02-13-09-16-26'
sift_iss_correspond_dir = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_correspondences_64/' + date_time



def readTestData(sift_test_root, iss_test_root):
    # get test list which store sift_patches
    # patch_file = list[submap_id][image_id]][patch_id]
    sift_test_list = []
    for submap_folder in sorted(os.listdir(sift_test_root)):
        submap_img_folder_list = []
        submap_folder_path = sift_test_root+submap_folder
        for image_folder in sorted(os.listdir(submap_folder_path)):
            img_folder_file_list = []
            img_folder_path = submap_folder_path + '/' +image_folder
            for img_file in sorted(os.listdir(img_folder_path)):
                if img_file.endswith('.png'):
                    img_folder_file_list.append(img_folder_path + '/'+ img_file)
            submap_img_folder_list.append(img_folder_file_list)
        sift_test_list.append(submap_img_folder_list)
        
    # get test list which store iss_volumes
    # iss_file = list[submap_id][iss_id]
    iss_test_list = []
    for submap_folder in sorted(os.listdir(iss_test_root)):
        submap_iss_file_list = []
        submap_folder_path = iss_test_root + submap_folder
        for iss_file in sorted(os.listdir(submap_folder_path)):
            if iss_file.endswith('.pcd'):
                submap_iss_file_list.append(submap_folder_path+'/'+iss_file)
        iss_test_list.append(submap_iss_file_list)
    
    return sift_test_list, iss_test_list


def getSIFTTestBatch(sift_test_list, batch_id):
    img_batch = np.zeros([batch_size, image_size, image_size,3], np.float32)    
    start_id = batch_id * batch_size
    end_id = (batch_id + 1) * batch_size 
    
    if (end_id > len(sift_test_list)):
        print("------ Error reading sift test batch!")
        return None
            
    # read batch
    data = Data(batch_size, image_size, pcl_size, None, None)
    list_batch = sift_test_list[start_id:end_id]
    for i in range(len(list_batch)):
        img =  cv2.imread(list_batch[i])
        img = data.img_augmentation(img)
        img_batch[i,:,:,:] = img
        
    return img_batch

def getISSTestBatch(iss_test_list, batch_id):
    pos_pcl_batch = np.zeros([batch_size, pcl_size, 3], np.float32)
    start_id = batch_id * batch_size
    end_id = (batch_id + 1) * batch_size 
    
    if (end_id > len(iss_test_list)):
        print("------ Error reading sift test batch!")
        return None
            
    # read batch
    data = Data(batch_size, image_size, pcl_size, None, None)
    list_batch = iss_test_list[start_id:end_id]
    for i in range(len(list_batch)):
        pos_pcl =  data.read_pcd(list_batch[i])
        # > 1024 points
        if pos_pcl.shape[0] > pcl_size:
            random_id = np.random.permutation(pos_pcl.shape[0])
            pos_pcl_batch[i, :, :] = pos_pcl[random_id[0:pcl_size]]
        else:
            pos_pcl_batch[i, 0:pos_pcl.shape[0], :] = pos_pcl      
            
    return pos_pcl_batch
  

def test(load_version, sift_test_list, iss_test_list, submap_id):
    print ('-----------------  START to test  -----------------')
        
    #sift_test_list = sift_test_list[submap_id-1][submap_image_id-1]
    iss_test_list = iss_test_list[submap_id-1]
    iss_test_file = "iss_test_list_txt/%03d.txt" % submap_id        
    with open(iss_test_file, 'w') as file:
        for i in range(len(iss_test_list)):
            file.write('%s\n' % iss_test_list[i])
    
    # define placeholder
    image_pl   = tf.placeholder(tf.float32, shape=[batch_size, image_size, image_size, 3])
    pos_pcl_pl = tf.placeholder(tf.float32, shape=[batch_size, pcl_size, 3])
    neg_pcl_pl = tf.placeholder(tf.float32, shape=[batch_size, pcl_size, 3])
    
    is_training = tf.placeholder(tf.bool)
    
    # build model
    print ('build model')
    with tf.device('/gpu:0'):   # use gpu 1 to forward
        with tf.variable_scope('image_branch') as scope:
            image_feature = vgg16(image_pl, is_training=True, output_dim=image_feature_dim,
                                  bn_decay=None)
        
        with tf.variable_scope('pointcloud_branch') as scope:
            pos_pcl_feature,_ = pointNet(pos_pcl_pl, pcl_feature_dim, is_training=is_training, 
                                       use_bn=False, bn_decay=None)
            scope.reuse_variables()
            neg_pcl_feature,_ = pointNet(neg_pcl_pl, pcl_feature_dim, is_training=is_training, 
                                       use_bn=False, bn_decay=None)

    saver = tf.train.Saver(tf.all_variables(), max_to_keep=None)  # tf.global_variables

    # run model
    print('run model...')
    config = tf.ConfigProto(log_device_placement=False, allow_soft_placement=True)
    config.gpu_options.allow_growth = True
    config.gpu_options.per_process_gpu_memory_fraction = 0.9
    with tf.Session(config=config) as sess:
        
        print('initialise model...')
        sess.run(tf.global_variables_initializer())
        print('   load model...')
        save_path = 'model/' + 'v3' + '/' + load_version +'_model.ckpt'
        saver.restore(sess, save_path)
        #restore_tf_model(sess)
        print("   Model loaded from: %s" % save_path)
                    
        # -------------------- evaluate model ---------------------
        print('**** Validate ...')
        print('   Compute image and pcl descriptors')
        

        iss_batch_num = len(iss_test_list) // batch_size 
        iss_test_num = iss_batch_num * batch_size      
        
        pcl_feature = np.zeros([iss_test_num, pcl_feature_dim])      
        
        # feed iss test list into the network
        batch_counter = 0
        print('-------- test iss --------------')
        for i in range(iss_batch_num):
            print("  *** iss progress: %d/%d" % (i, iss_batch_num))
            pcl_batch = getISSTestBatch(iss_test_list,i)
            feed_dict = {pos_pcl_pl:pcl_batch, is_training: False}
            pcl_batch_feature = sess.run(pos_pcl_feature, feed_dict=feed_dict)
            pcl_feature[batch_counter: batch_counter+pcl_batch_feature.shape[0],:] = pcl_batch_feature
            batch_counter += pcl_batch_feature.shape[0] 
            
        print('---------- test sift ----------')
        sift_submap_test_list = sift_test_list[submap_id-1]     # all images
        for k in range(len(sift_submap_test_list)):
            sift_test_list = sift_submap_test_list[k]       # image id: i+1            
            cam_id = sift_test_list[0].split('/')[-2]   # expected 'cam1_xxx'
            # record test_list for checking
            sift_test_file = "sift_test_list_txt/%03d_%s.txt" % (submap_id, cam_id)
            with open(sift_test_file, 'w') as file:
                for i in range(len(sift_test_list)):
                    file.write('%s\n' % sift_test_list[i])
            
            # test the patches from one image in the submap
            sift_batch_num = len(sift_test_list) // batch_size
            sift_test_num = sift_batch_num * batch_size
            img_feature = np.zeros([sift_test_num, image_feature_dim])
            
            # feed sift test list into the network
            batch_counter = 0
            print("  *** image id: %d/%d" % (k,len(sift_submap_test_list)))
            for i in range(sift_batch_num):
                #print("  *** image id: %d/%d, batch id: %d/%d" % (k, len(sift_submap_test_list), i, sift_batch_num))
                img_batch = getSIFTTestBatch(sift_test_list, i)
                #print img_batch.shape
                feed_dict = {image_pl:img_batch, is_training: False}
                img_batch_feature = sess.run(image_feature, feed_dict=feed_dict)
                #print type(img_batch_feature)
                img_feature[batch_counter: batch_counter+img_batch_feature.shape[0],:] = img_batch_feature
                batch_counter += img_batch_feature.shape[0]          
            
            # compute distance array between img_feature and pcl_feature
            img_vec = np.sum(np.multiply(img_feature, img_feature), axis=1, keepdims=True)
            pcl_vec = np.sum(np.multiply(pcl_feature, pcl_feature), axis=1, keepdims=True)
            dist_array = img_vec + np.transpose(pcl_vec) - 2*np.matmul(img_feature, np.transpose(pcl_feature))
            print("  image patch num: %d, submap pcl num: %d" % (dist_array.shape[0], dist_array.shape[1]))
        
            # find correspondences and record
            # img_pcl_correspondences = [];
            cam_id = sift_test_list[0].split('/')[-2]
            txt_folder = "%s/%03d" % (sift_iss_correspond_dir, submap_id)
            if not os.path.exists(txt_folder):
                os.makedirs(txt_folder)
            txt_file_path = "%s/%s.txt" % (txt_folder, cam_id)
            top_k = 10
            with open(txt_file_path, "w") as file:
                for i in range(dist_array.shape[0]):
                    #min_dist_id = np.argmin(dist_array[i,:])
                    min_dist_id  = np.argsort(dist_array[i,:])[:top_k]
                    idx = np.concatenate((np.array([i+1]), min_dist_id+1))
                    #print(idx)
                    idx=idx.reshape(1, idx.shape[0])
                    np.savetxt(file, idx,fmt='%d')
    
if __name__ == '__main__':
    
    load_version = 'v1_6_5000'
    
#    date_time = '2014-07-14-15-16-36'
    
    submap_id = 121    #116-121
    
    sift_test_root = '/media/mengdan/data3/robotcar/grasshopper/sift_patch_test/' + date_time + '/'
    iss_test_root = '/media/mengdan/data3/robotcar/grasshopper/iss_volume_test/' + date_time + '/'  

    # read test data
    sift_test_list, iss_test_list = readTestData(sift_test_root, iss_test_root)    
    # test 
    test(load_version, sift_test_list, iss_test_list, submap_id)
    