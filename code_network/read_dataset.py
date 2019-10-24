#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat May 19 20:23:09 2018

@author: mengdan
"""

import os
import numpy as np
import random
import cv2
import math
import pypcd
from numpy import linalg as LA

class DataEntry:
    submap_id = None
    cam_id = None
    sift_filename = None
    iss_filename = None
    
class Data:
    
    def __init__(self, batch_size, image_size, pcl_size, train_list, test_list):
        
        self.__batch_size = batch_size
        self.__route_id = 0         # which run
        self.__batch_id = 0         # which batch
        self.__test_batch_id = 0
        self.__image_size = image_size
        self.__pcl_size = pcl_size
        #self.__train_list = train_list
        self.train_list = train_list
        self.__test_list = test_list
        
        #self.__data_root = '/media/mengdan/Samsung850Pro/robotcar/grasshopper'
        self.__data_root = '/media/mengdan/data3/robotcar/grasshopper'
        
        
    def getTrainBatch(self):
        img_batch = np.zeros([self.__batch_size, self.__image_size,self.__image_size,3], np.float32)
        pos_pcl_batch = np.zeros([self.__batch_size, self.__pcl_size, 3], np.float32)
        neg_pcl_batch = np.zeros([self.__batch_size, self.__pcl_size, 3], np.float32)
        
        start_id = self.__batch_id * self.__batch_size
        end_id = (self.__batch_id + 1) * self.__batch_size
        
        round_count = 0
        while (end_id > len(self.train_list[self.__route_id]) and 
               round_count < len(self.train_list)):
            self.__route_id += 1        # jump to next route
            
            if self.__route_id >= len(self.train_list):
                self.__route_id = 0
                self.__batch_id += 1
            
            round_count += 1
        
        # end of this epoch, return None
        # if round_count == len(self.__train_list[self.__route_id]):
        if round_count == len(self.train_list):
            print("*** End of epoch!")
            self.__route_id = 0         # which run
            self.__batch_id = 0         # which batch
            return None, None, None
        
        
        # read batch
        list_batch = self.train_list[self.__route_id][start_id:end_id]
        for i in range(self.__batch_size):
            anchor_img = cv2.imread(list_batch[i].sift_filename)
            anchor_img = self.img_augmentation(anchor_img)
            
            pos_pcl = self.read_pcd(list_batch[i].iss_filename)
            
            #print list_batch[i].sift_filename
            #print list_batch[i].iss_filename
            
            # find negative pcl
            while True:
                random_id = random.randint(0, len(self.train_list[self.__route_id]) - 1)
                random_submap_id = self.train_list[self.__route_id][random_id].submap_id
                random_cam_id = self.train_list[self.__route_id][random_id].cam_id
                if (abs(random_submap_id - list_batch[i].submap_id) >= 2 or 
                    random_cam_id != list_batch[i].cam_id):
                    break

            neg_pcl = self.read_pcd(self.train_list[self.__route_id][random_id].iss_filename)
            
            
            img_batch[i, :, :, :] = anchor_img
            
            if pos_pcl.shape[0] > self.__pcl_size:
                random_id = np.random.permutation(pos_pcl.shape[0])
                pos_pcl_batch[i, :, :] = pos_pcl[random_id[0:self.__pcl_size]]
            else:
                pos_pcl_batch[i, 0:pos_pcl.shape[0], :] = pos_pcl
            
            if neg_pcl.shape[0] > self.__pcl_size:
                random_id = np.random.permutation(neg_pcl.shape[0])
                neg_pcl_batch[i, :, :] = neg_pcl[random_id[0:self.__pcl_size]]
            else:
                neg_pcl_batch[i, 0:neg_pcl.shape[0], :] = neg_pcl
        
        # pcl pre-processing
        pos_pcl_batch = self.pcl_augmentation(pos_pcl_batch)
        neg_pcl_batch = self.pcl_augmentation(neg_pcl_batch)
        
        # update route_id and batch_id
        self.__route_id += 1
        if self.__route_id >= len(self.train_list):
            self.__route_id = 0
            self.__batch_id += 1     
            
        return img_batch, pos_pcl_batch, neg_pcl_batch
            
#    def getTestBatch(self):
#        img_batch = np.zeros([self.__batch_size, self.__image_size,self.__image_size,3], np.float32)
#        pos_pcl_batch = np.zeros([self.__batch_size, self.__pcl_size, 3], np.float32)
#        neg_pcl_batch = np.zeros([self.__batch_size, self.__pcl_size, 3], np.float32)
#        
#        start_id = self.__batch_id * self.__batch_size
#        end_id = (self.__batch_id + 1) * self.__batch_size - 1
#        
#        round_count = 0
#        while (end_id >= len(self.__test_list[self.__route_id]) and 
#               round_count < len(self.__test_list)):
#            self.__route_id += 1        # jump to next route
#            
#            if self.__route_id >= len(self.__test_list):
#                self.__route_id = 0
#                self.__batch_id += 1
#            
#            round_count += 1
#        
#        # end of this epoch, return None
#        # if round_count == len(self.__test_list[self.__route_id]):
#        if round_count == len(self.__test_list):
#            print("*** End of epoch!")
#            self.__route_id = 0         # which run
#            self.__batch_id = 0         # which batch
#            return None, None, None
#        
#        
#        # read batch
#        list_batch = self.__test_list[self.__route_id][start_id:end_id]
#        for i in range(self.__batch_size):
#            anchor_img = cv2.imread(list_batch[i].sift_filename)
#            anchor_img = self.img_augmentation(anchor_img)
#            pos_pcl = self.read_pcd(list_batch[i].iss_filename)
#            
#            # find negative pcl
#            while True:
#                random_id = random.randint(0, len(self.__test_list[self.__route_id]) - 1)
#                random_submap_id = self.__test_list[self.__route_id][random_id].submap_id
#                random_cam_id = self.__test_list[self.__route_id][random_id].cam_id
#                if (abs(random_submap_id - list_batch[i].submap_id) >= 2 or 
#                    random_cam_id != list_batch[i].cam_id):
#                    break
#            neg_pcl = self.read_pcd(self.__test_list[self.__route_id][random_id].iss_filename)
#            
#            if pos_pcl.shape[0] > self.__pcl_size:
#                random_id = np.random.permutation(pos_pcl.shape[0])
#                pos_pcl_batch[i, :, :] = pos_pcl[random_id[0:self.__pcl_size]]
#            else:
#                pos_pcl_batch[i, 0:pos_pcl.shape[0], :] = pos_pcl
#            
#            if neg_pcl.shape[0] > self.__pcl_size:
#                random_id = np.random.permutation(neg_pcl.shape[0])
#                neg_pcl_batch[i, :, :] = neg_pcl[random_id[0:self.__pcl_size]]
#            else:
#                neg_pcl_batch[i, 0:neg_pcl.shape[0], :] = neg_pcl
#                
#        # pcl pre-processing
#        pos_pcl_batch = self.pcl_augmentation(pos_pcl_batch)
#        neg_pcl_batch = self.pcl_augmentation(neg_pcl_batch)
#        
#        # update route_id and batch_id
#        self.__route_id += 1
#        if self.__route_id >= len(self.__test_list):
#            self.__route_id = 0
#            self.__batch_id += 1     
#        
#        return img_batch, pos_pcl_batch, neg_pcl_batch
    
    def getTestBatch(self):
        img_batch = np.zeros([self.__batch_size, self.__image_size,self.__image_size,3], np.float32)
        pos_pcl_batch = np.zeros([self.__batch_size, self.__pcl_size, 3], np.float32)
        
        start_id = self.__test_batch_id * self.__batch_size
        end_id = (self.__test_batch_id + 1) * self.__batch_size 
        
        if (end_id > len(self.__test_list)):
            self.__test_batch_id = 0         # which batch
            return None, None
                
        # read batch
        list_batch = self.__test_list[start_id:end_id]
        for i in range(self.__batch_size):
            anchor_img = cv2.imread(list_batch[i].sift_filename)
#            if i==0:
#                print start_id ,i,list_batch[i].sift_filename
            anchor_img = self.img_augmentation(anchor_img)
            img_batch[i, :, :, :] = anchor_img
            pos_pcl = self.read_pcd(list_batch[i].iss_filename)
            
            # > 1024 points
            if pos_pcl.shape[0] > self.__pcl_size:
                random_id = np.random.permutation(pos_pcl.shape[0])
                pos_pcl_batch[i, :, :] = pos_pcl[random_id[0:self.__pcl_size]]
            else:
                pos_pcl_batch[i, 0:pos_pcl.shape[0], :] = pos_pcl
        
        # pre-processing
        #pos_pcl_batch = self.pcl_augmentation(pos_pcl_batch)
        
        # update route_id and batch_id
        self.__test_batch_id += 1     
        
        return img_batch, pos_pcl_batch
    
    
    def read_pcd(self, pcd_filename):
        """
        Read pcd file (pointcloud)
        """
        if not os.path.isfile(pcd_filename):
            print("  File %s does not exist!" % (pcd_filename))
            return
        
        pcl = pypcd.PointCloud.from_path(pcd_filename)
        x = pcl.pc_data['x']
        y = pcl.pc_data['y']
        z = pcl.pc_data['z']
        
#        # zero-mean 
#        x -= x.mean()
#        y -= y.mean()
#        z -= z.mean()          

        pcl = np.transpose(np.array([x,y,z],dtype=np.float32))
        
#        # unit-sphere normalization
#        max_radius = max(LA.norm(pcl, axis=1)) 
                
#        pcl /= max_radius

        return pcl
            
    
    def img_augmentation(self, img):
        '''Do image augmentation: random rotation (0, 90, 180, 270 degrees), random scale + centre crop
            The output format of image is np.float32
        '''
#        # random rotate
#        rand_rotate = math.floor(random.random() * 4) * 90
#        rot_matrix = cv2.getRotationMatrix2D((img.shape[1] / 2, img.shape[0] / 2), rand_rotate, 1)
#        img = cv2.warpAffine(img, rot_matrix, (img.shape[1], img.shape[0]))

#        rand_scale = random.random() * 0.5 + 1.0
#        if rand_scale < 1.0:
#            # random scale
#            img = cv2.resize(img, (int(img.shape[1] * rand_scale), int(img.shape[0] * rand_scale)),
#                             interpolation=cv2.INTER_CUBIC)
#            # centre crop
#            crop_start = int((img.shape[1] - 512.0) / 2.0)
#            crop_end = crop_start + 512
#            img = img[crop_start: crop_end, crop_start: crop_end, :]
        
        img = cv2.resize(img, (self.__image_size, self.__image_size),
                         interpolation=cv2.INTER_CUBIC)
        img = img.astype(np.float32)
        
        # for using VGG pre-trained model
        img[:, :, 0] -= 103.939  # Blue
        img[:, :, 1] -= 116.779  # Green
        img[:, :, 2] -= 123.6    # Red

        return img
    
    def pcl_augmentation(self, pcl_batch):
        '''
        Rotate and jitter pcl batch
        '''
        pcl_batch = self.rotate_point_cloud(pcl_batch)
        pcl_batch = self.jitter_point_cloud(pcl_batch)
        
        return pcl_batch          
    

    def rotate_point_cloud(self, batch_data):
        """ Randomly rotate the point clouds to augument the dataset
            rotation is per shape based along up direction
            Input:
              BxNx3 array, original batch of point clouds
            Return:
              BxNx3 array, rotated batch of point clouds
        """
        rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
        for k in range(batch_data.shape[0]):
            rotation_angle = np.random.uniform() * 2 * np.pi
            cosval = np.cos(rotation_angle)
            sinval = np.sin(rotation_angle)
            rotation_matrix = np.array([[cosval, 0, sinval],
                                        [0, 1, 0],
                                        [-sinval, 0, cosval]])
            shape_pc = batch_data[k, ...]
            rotated_data[k, ...] = np.dot(shape_pc.reshape((-1, 3)), rotation_matrix)
        return rotated_data
    

    def rotate_point_cloud_by_angle(self, batch_data, rotation_angle):
        """ Rotate the point cloud along up direction with certain angle.
            Input:
              BxNx3 array, original batch of point clouds
            Return:
              BxNx3 array, rotated batch of point clouds
        """
        rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
        for k in range(batch_data.shape[0]):
            #rotation_angle = np.random.uniform() * 2 * np.pi
            cosval = np.cos(rotation_angle)
            sinval = np.sin(rotation_angle)
            rotation_matrix = np.array([[cosval, 0, sinval],
                                        [0, 1, 0],
                                        [-sinval, 0, cosval]])
            shape_pc = batch_data[k, ...]
            rotated_data[k, ...] = np.dot(shape_pc.reshape((-1, 3)), rotation_matrix)
        return rotated_data
    
    
    def jitter_point_cloud(self, batch_data, sigma=0.01, clip=0.05):
        """ Randomly jitter points. jittering is per point.
            Input:
              BxNx3 array, original batch of point clouds
            Return:
              BxNx3 array, jittered batch of point clouds
        """
        B, N, C = batch_data.shape
        assert(clip > 0)
        jittered_data = np.clip(sigma * np.random.randn(B, N, C), -1*clip, clip)
        jittered_data += batch_data
        return jittered_data
        
    def pcl_unit_normalization(self, batch_data):
        """ Normalize the pointcloud to unit sphere
            Input: 
                BxNx3 array, original batch of pointcloud 
            Return:
                BxNx3 array, normalized pointcloud
        """
        


#def splitTrainTest(dataset_dir):
#    """
#    This function split the dataset into train (80%) and test(20%)
#    """
#    dataset_train = []
#    dataset_test = []
#    for filename in os.listdir(dataset_dir):
#        if filename.endswith('.txt'):
#            print filename            
#            with open(dataset_dir + '/' + filename) as file:
#                sub_dataset_train = []
#                sub_dataset_test = []
#                all_data = file.readlines()
#                file_num = len(all_data)
#                train_num = int(file_num*0.8)
#                
#                num = 0
#                for line in all_data:
#                    data = line.split()
#                    
#                    entry = DataEntry()
#                    entry.submap_id = int(data[0])
#                    entry.cam_id = int(data[1])
#                    entry.sift_filename = data[2]
#                    entry.iss_filename = data[3]
#                    
#                    if num<train_num:
#                        sub_dataset_train.append(entry)
#                        num+=1
#                    else:                        
#                        sub_dataset_test.append(entry)
#                
#                dataset_train.append(sub_dataset_train)
#                dataset_test.append(sub_dataset_test)     
#    
#    print 'splitTrainTest::dataset_train: length =', len(dataset_train)
#    for i in range(len(dataset_train)):
#        print '   ', len(dataset_train[i])
#    print 'splitTrainTest::dataset_test: length =', len(dataset_test)
#    for i in range(len(dataset_test)):
#        print '   ', len(dataset_test[i])
#            
#    return dataset_train, dataset_test

def splitTrainTest(dataset_dir):
    """
    This function split the dataset into train (80%) and test(20%)
    """
    dataset_train = []
    dataset_test = []

    total_train_submaps = 0
    total_test_submaps = 0
    for filename in os.listdir(dataset_dir):
        if filename.endswith('.txt'):
            print filename            
            with open(dataset_dir + '/' + filename) as file:
                sub_dataset_train = []
                sub_dataset_test = []
                all_data = file.readlines()
                
                # get all submapId in each run
                unique_submap_id = []
                for line in all_data:
                    data = line.split()
                    entry = DataEntry()
                    entry.submap_id = int(data[0])
                    if entry.submap_id not in unique_submap_id:
                        unique_submap_id.append(entry.submap_id)
                
                # get last 10% submaps for testing
                test_submap_num = int(len(unique_submap_id) * 0.1)
                #test_submap_id = unique_submap_id[-test_submap_num:]
                train_submap_num = len(unique_submap_id) - test_submap_num
                train_submap_id = unique_submap_id[0:train_submap_num]

                total_train_submaps += train_submap_num
                total_test_submaps += test_submap_num

                test_submap_id = {}
                for i in range(test_submap_num):
                    test_submap_id[unique_submap_id[train_submap_num + i]] = i
                    sub_dataset_test.append([])
                
                # get train and test list
                #num = 0
                for line in all_data:
                    data = line.split()
                    
                    entry = DataEntry()
                    entry.submap_id = int(data[0])
                    entry.cam_id = int(data[1])
                    entry.sift_filename = data[2]
                    entry.iss_filename = data[3]
                    
                    if entry.submap_id in train_submap_id:
                        sub_dataset_train.append(entry)
                    elif entry.submap_id in test_submap_id: 
                        index = test_submap_id[entry.submap_id]
                        sub_dataset_test[index].append(entry)
                    else:
                        print("Invalid submap_id in function splitTrainTest! YINGYINGYING")
                        exit(0)                
                
                dataset_train.append(sub_dataset_train)
                dataset_test.append(sub_dataset_test)    

    total_train_pairs = 0
    total_test_pairs = 0
    print 'splitTrainTest::dataset_train: length =', len(dataset_train)
    for i in range(len(dataset_train)):
        print '   ', len(dataset_train[i])
        total_train_pairs += len(dataset_train[i])
    print 'splitTrainTest::dataset_test: length =', len(dataset_test)
    for i in range(len(dataset_test)):
        print '   ', len(dataset_test[i])
        for j in range(len(dataset_test[i])):
            total_test_pairs += len(dataset_test[i][j])

    print 'total train/test submaps: ', total_train_submaps, total_test_submaps
    print 'total train/test image_pcl pairs: ', total_train_pairs, total_test_pairs
    return dataset_train, dataset_test

    
def shuffleTrainset(train_list):
    '''
    Shuffle train set
    '''
    random.shuffle(train_list)
    shuffled_train_list = []
    for i in range(len(train_list)):
        random.shuffle(train_list[i])
        shuffled_train_list.append(train_list[i])
    
    return shuffled_train_list

        
        
        
        
    
    
    
    
    
    
    
    
    