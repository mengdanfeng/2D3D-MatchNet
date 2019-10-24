#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  6 21:46:30 2018

@author: mengdan
"""
import numpy as np
import matplotlib.pyplot as plt
import math

#txt_file = '/media/mengdan/data3/robotcar/grasshopper/results/2014-07-14-15-16-36/084.txt'
txt_file = '/media/mengdan/data3/robotcar/grasshopper/results/2014-06-26-09-53-12/043.txt'
line_counter = 0
valid_pose_number = 0

t_total_bins = 11
t_counter = np.zeros((t_total_bins,))
t_accuracy = np.zeros((t_total_bins,))
t_bin_size = 1

r_total_bins = 10
r_counter = np.zeros((r_total_bins,))
r_accuracy = np.zeros((r_total_bins,))
r_bin_size = 5


with open(txt_file, 'r') as file:
    for line in file:
        # pose start from line 5
        line_counter += 1        
        
        # read the second line, the third number
        if line_counter == 2:
            data = line.split(', ')
            valid_image_num = data[2]
#            print valid_image_num
        
        if line_counter< 5:
            continue
        
        data = line.split(', ')
        # skip invalid pose
        if data[2] == 'None\n':
            #t_counter[-1] += 1
            continue

        valid_pose_number += 1
        # count t-error and r-error
        pred_err_t = float(data[2])
        pred_err_r = float(data[3])*180/math.pi
        
        if pred_err_t > 10:
            print("big t_error: %.2f" % (pred_err_t))
            t_counter[-1] += 1
        else:
            t_counter[int(pred_err_t/t_bin_size)] += 1
        
        if pred_err_r > 30:
            print("big r_error: %.2f" % pred_err_r)
            r_counter[-1] += 1
        else:
            print pred_err_r
            r_counter[int(pred_err_r/r_bin_size)] += 1
    
    # compute histogram of accuracy
    t_accumutive_accu = 0.0
    r_accumutive_accu = 0.0
    t_accuracy[0] = 0.0
    r_accuracy[0] = 0.0
    for i in range(1, t_counter.shape[0]):
        t_accuracy[i] = t_counter[i-1] + t_accuracy[i-1]        
    
    for i in range(1,r_counter.shape[0]):
        r_accuracy[i] = r_counter[i-1] + r_accuracy[i-1]
        
    t_accuracy /= float(valid_image_num)
    r_accuracy /= float(valid_image_num)
        
    # plot t error
    fig = plt.figure(figsize=(5.0, 2.0))
    plt.plot([0,1,2,3,4,5,6,7,8,9,10], t_accuracy,label='Ours')
    plt.plot([0,1,2,3,4,5,6,7,8,9,10], [0.0,0.0018, 0.0085,0.0165,0.0254,0.0303,0.0374,0.0432,0.0490,0.0539,0.0606],label='ORB-SLAM2')
    plt.xticks([0,1,2,3,4,5,6,7,8,9,10])
    plt.yticks([0, 0.2, 0.4, 0.6])
    plt.grid(True, which='major', linestyle='dashed', alpha=0.5)
    
    plt.axis([0, 10, 0.0, 0.6])
    plt.xlabel('Positional error (m)',fontsize=15)
    plt.rc('xtick', labelsize=15) 
    plt.rc('ytick',labelsize=15)
    plt.legend(bbox_to_anchor=(0.72, 0.83), loc='upper left', 
       ncol=1, fontsize=8, borderaxespad=0.0)
    
    # plot r error
    fig = plt.figure(figsize=(5.0, 2.0))
    plt.plot([0,5,10,15,20,25,30,35,40,45], r_accuracy, label='Ours')
    plt.plot([0,5,10,15,20,25,30,35,40,45], [0, 0.0302895322939866,	0.0605790645879733,0.0846325167037862,0.117594654788419,	0.176391982182628,	0.189309576837416,	0.200890868596882,	0.211135857461025,	0.222271714922049], label='ORB-SLAM2')
    plt.xticks([0,5,10,15,20,25,30,35,40,45])
    plt.yticks([0, 0.2, 0.4, 0.6])
    plt.grid(True, which='major', linestyle='dashed', alpha=0.5)
    
    plt.axis([0, 45, 0.0, 0.6])
    plt.xlabel('Angular error (degree)',fontsize=15)
    plt.rc('xtick', labelsize=15) 
    plt.rc('ytick',labelsize=15)
    plt.legend(bbox_to_anchor=(0.72, 0.83), loc='upper left', 
       ncol=1, fontsize=8, borderaxespad=0.0)
#    plt.ylabel('Recall Accuracy at Top-K')
#    t_accuracy[-1] = (285 - valid_pose_number + t_counter[-1])/285
#    r_accuracy[-1] = (285 - valid_pose_number + r_counter[-1])/285
        
    # record accuracy
#    print t_accuracy
#    print r_accuracy
    
    # plot t_accuracy histogram
#    plt.bar(range(len(t_accuracy)), t_accuracy)
#    # plt.grid()
#    # plt.title()
#    plt.xlabel('Position error (m)')
#    plt.xticks(range(len(t_accuracy)))
#    plt.ylabel('Percentage')
    #plt.show()
    
    # plot r_accuracy histogram
#    plt.bar(range(len(r_accuracy)), r_accuracy)
#    plt.xlabel('Angular error (degree)')
#    plt.xticks(range(len(r_accuracy))*5)
#    plt.ylabel('Percentage')
#    plt.show()    
#    
        
        
        
    

