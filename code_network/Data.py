#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May 18 22:22:43 2018

@author: mengdan
"""

import os
import numpy as np
import random
import cv2
#import math

def splitTrainTest(dataset_dir):
    """
    This function split the dataset into train (80%) and test(20%)
    """
    dataset_train = []
    dataset_test = []
    for filename in os.listdir(dataset_dir):
        with open(filename) as file:
            sub_dataset_train = []
            sub_dataset_test = []
            file_num = len(file.readlines())
            train_num = int(file_num*0.8)
            
            num = 0
            for line in file:
                data = line.split()
                
                entry = DataEntry()
                entry.submap_id = int(data[0])
                entry.cam_id = int(data[1])
                entry.sift_filename = data[2]
                entry.iss_filename = data[3]
                
                if num<train_num:
                    sub_dataset_train.append(entry)
                    num+=1
                else:                        
                    sub_dataset_test.append(entry)
            
            dataset_train.append(sub_dataset_train)
            dataset_test.append(sub_dataset_test)     
            
    return dataset_train, dataset_test

def getDatasetMaxBatch(train_list, test_list):
    """
    This function returns the maximum batch size for each sub_list in data_list
    """
    trainset_batch_num = []
    testset_batch_num = []
    
    for i in range(len(train_list)):
        subset_list = train_list[i]
        subset_batch_num = len(subset_list) // batch_size
        trainset_batch_num.append(subset_batch_num)
        
    for i in range(len(test_list)):
        subset_list = test_list[i]
        subset_batch_num = len(subset_list) // batch_size
        testset_batch_num.append(subset_batch_num)                  
    return trainset_batch_num, testset_batch_num

def shuffleTrainset(train_list):
    """
    This function shuffles trainset list at per epoch
    """
    shuffled_train_list = []
    for i in range(len(train_list)):
        sub_list = train_list[i]
        random.shuffle(sub_list)
        shuffled_train_list.append(sub_list)
    
    return shuffled_train_list


class DataEntry:
    submap_id = None
    cam_id = None
    sift_filename = None
    iss_filename = None
    

class Dataset:
    
    def __init__(self, batch_size, train_list, train_list_batch_num, 
                 test_list, test_list_batch_num):
               
        self.__train_list = train_list
        self.__test_list = test_list
        self.__train_list_batch_num = train_list_batch_num
        self.__test_list_batch_num = test_list_batch_num
        
        self.__batch_size = batch_size
        self.__batch_id = 0   
        self.__train_list_id = 0
        self.__test_list_id = 0
    
    def getBatch(self, image_size, point_number):
        
        if self.__batch_id > self.__train_list_batch_num(self.__train_list_id) 
            & self.__train_list_id < len(self.__train_list):
            self.__train_list_id += 1
            self.__batch_id = 0
        else:
            print("End of this epoch!\n")
            break
                
        img_batch = np.zeros([self.__batch_size, image_size,image_size,3])
        pcl_batch = np.zeros([self.__batch_size, point_number, 3])
        
        # random.shuffle(len(self.__dataset))
        
        
        
        
        


            
            
            
            
            
        
        