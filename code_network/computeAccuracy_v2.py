#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 21:00:48 2018

@author: mengdan
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 14:32:16 2018

@author: mengdan
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 20:04:26 2018

@author: mengdan
"""

'''
This script is modified based on computeAccuracy.py
All sift_uv and top 10 iss_xyz will be computed using solvePnPRansac
'''
import numpy as np
import os
import sys
import cv2
from scipy.spatial import distance

# --------- Parameters ------------
invaild_pairs_threshold = 4

top_k = 5

gt_correspondence_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_corr_gt/2014-06-26-09-53-12'
pred_correspondence_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_correspondences/2014-06-26-09-53-12'
gt_pose_root = '/media/mengdan/data3/robotcar/grasshopper/cam_global_poses/2014-06-26-09-53-12'

sift_iss_idx_value_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_index_value/2014-06-26-09-53-12'


# -------- camera parameters ------------
# camera intrinsics
camera_matrix_left = np.array([[400, 0  , 500.107605000000], 
                               [0  , 400, 511.461426000000],
                               [0  , 0  , 1]])

camera_matrix_right = np.array([[400, 0,   502.503754000000],
                               [0  ,	400, 490.259033000000],
                               [0	  , 0,	1]])

# T_ins_to_camera/G_camera_ins
T_ins_to_camera_left = np.array([[0.313485785451577, -0.941953894385083, 0.120206169444769, -0.612518528650024],
						     [0.929455016766250, 0.278437884954273, -0.242044863672503, -0.238809567784806],
						     [0.194525150373564, 0.187603851439589, 0.962790091763088, 1.04417692175197],
						     [0, 0, 0, 1]])

T_ins_to_camera_right = np.array([[0.289549795371260, 0.947179007633010,	0.137887067920849,	-0.505997744800505],
                                 [-0.934903387271299,	0.248978920275067,	0.252913332442781,	0.276521207212685],
                                 [0.205223225949483,	-0.202142090515581,	0.957612658005771,	1.01794411533368],
                                 [0,	0,	0,	1]])

# T_camera_to_cameraa/G_camera_image
T_camera_to_cameraa_left = np.array([[0, 1, 0, 0], 
	                            [0, 0, 1, 0],
	                            [1, 0, 0, 0], 
	                            [0, 0, 0, 1]])

T_camera_to_cameraa_right = np.array([[0,	0,	1,	0],
                                     [1,	0,	0,	0],
                                     [0,	1,	0,	0],
                                     [0,	0,	0,	1]])


# ---------------------------------

class ImgCorrespondence:
    
    num_of_pairs = None
    
    sift_id = None
    iss_id = None
    
    def __init__(self):
        self.sift_id = []
        self.iss_id = []


def readCorrespondences(submap_id):
    # gt 2d-3d correspondences
    submap_id = str('%03d' % submap_id)
    filenames = os.listdir(gt_correspondence_root + '/' + submap_id)
    gt_corr = [None] * len(filenames)
    id_base = len(filenames) / 2
    for filename in filenames: 
        with open(gt_correspondence_root + '/' + submap_id + '/' + filename, 'r') as file:
            img_corr = ImgCorrespondence()
            img_corr.num_of_pairs = int(file.readline())
            
            # ignore it if the number of pairs is less than a threshold
            if img_corr.num_of_pairs < invaild_pairs_threshold:
                continue
            
            for i in range(img_corr.num_of_pairs):
                line = file.readline()
                data = line.split()
                img_corr.sift_id.append(int(data[0]))
                img_corr.iss_id.append(int(data[1]))
                
            cam_id = int(filename[3]) - 1
            img_id = int(filename[5:8]) - 1
            gt_corr[cam_id * id_base + img_id] = img_corr
            
    
    # predicted 2d-3d correspondences
    filenames = os.listdir(pred_correspondence_root + '/' + submap_id)
    pred_corr = [None] * len(filenames)
    id_base = len(filenames) / 2
    for filename in filenames: 
        #print ('read ' + pred_correspondence_root + '/' + submap_id + '/' + filename)
        with open(pred_correspondence_root + '/' + submap_id + '/' + filename, 'r') as file:
            img_corr = ImgCorrespondence()
            
            for line in file:
                data = line.split()
                img_corr.sift_id.append(int(data[0]))
                img_corr.iss_id.append([int(x) for x in data[1 : top_k+1]])
                
            cam_id = int(filename[3]) - 1
            img_id = int(filename[5:8]) - 1
            pred_corr[cam_id * id_base + img_id] = img_corr
    
    return gt_corr, pred_corr


def readGTPose(submap_id):
    submap_id = str('%03d' % submap_id)
    filename1 = gt_pose_root + '/' + submap_id + '/cam1_poses.txt'
    filename2 = gt_pose_root + '/' + submap_id + '/cam2_poses.txt'
    gt_poses = []
    with open(filename1, 'r') as file:
        for line in file:
            data = line.split(',')
            gt_poses.append([float(x) for x in data])
    with open(filename2, 'r') as file:
        for line in file:
            data = line.split(',')
            gt_poses.append([float(x) for x in data])
            
    #print('length of gt_poses: %d' % len(gt_poses))
    return gt_poses


def readSIFTUV(submap_id):
    # read sift_uv
    submap_id = str('%03d' % submap_id)
    filenames = os.listdir(sift_iss_idx_value_root + '/' + submap_id)
    sift_uv = [None] * (len(filenames)-1)
    id_base = (len(filenames)-1) / 2
#    print id_base
#    count = []
    for filename in filenames:
        if filename.startswith('cam'):
            img_sift_uv = []
            cam_id = int(filename[3]) - 1
            img_id = int(filename[5:8]) - 1
            with open(sift_iss_idx_value_root + '/' + submap_id+'/'+filename, 'r') as file:
                #print(sift_iss_idx_value_root + '/' + submap_id+'/'+filename)
                for line in file:
                    #print(line)
                    data = line.split(',')
                    img_sift_uv.append([float(x) for x in data[1:3]])
            sift_uv[cam_id*id_base+img_id] = img_sift_uv   
#            count.append(cam_id*id_base+img_id)
#    print(len(set(count)))
    return sift_uv                
            
   
def readISSXYZ(submap_id):
    # read iss_xyz
    submap_id = str('%03d' % submap_id)
    iss_xyz_file = sift_iss_idx_value_root + '/' + submap_id + '/' + 'iss_'+submap_id+'.txt'
    iss_xyz = []
    with open(iss_xyz_file, 'r') as file:
        for line in file:
            data = line.split(',')
            iss_xyz.append([float(x) for x in data[1:4]])        
    return iss_xyz

def getPnPSolution(points_2d, points_3d, camera_matrix, T_ins_to_camera, T_camera_to_cameraa):
    '''
    This function estimates camera pose using EPnP
    Input: 
        image_points: Nx2 array
        world_points: Nx3 array
        camera_matrix: 3x3 camera matrix
    Output:
        camera_pose
    '''
    # estimate ins 1' pose in camera coordinates
    points_2d = np.expand_dims(points_2d, axis=1)
    points_3d = np.expand_dims(points_3d, axis=1)
    pose = cv2.solvePnPRansac(points_3d, points_2d, camera_matrix, None, None, None, False, iterationsCount=100000, flags=cv2.SOLVEPNP_EPNP)
    
    if pose[0] is False:
        return None
    
    #print pose: pose[0]: True/False, pose[1]:rvec, pose[2]:tvec, pose[3]:inlier_ids
    #pose = cv2.solvePnP(points_3d, points_2d, camera_matrix, None, None, None, False,flags=cv2.SOLVEPNP_EPNP)

    # ins1 -> camera (origin ins 1's pose in camera coordinate)
    rot_matrix, _ = cv2.Rodrigues(pose[1])
    T_ins1_to_camera = np.zeros([4, 4])
    T_ins1_to_camera[0:3, 0:3] = rot_matrix
    T_ins1_to_camera[0:3, 3] = np.squeeze(pose[2])
    T_ins1_to_camera[3, 3] = 1.0

    T = np.matmul(np.linalg.inv(T_ins1_to_camera), np.matmul(T_camera_to_cameraa, T_ins_to_camera))
    
    # 4x4 matrix to rvec, tvec
    tvec = T[0:3,3]
    rvec,_ = cv2.Rodrigues(T[0:3,0:3])
    pred_pose = np.zeros([6,])
    pred_pose[0:3] = tvec
    pred_pose[3:6] = np.squeeze(rvec)
    return pred_pose    
    
    
def computeAccuracy(submap_id):
    print("--- read correspondences---")
    gt_corr, pred_corr = readCorrespondences(submap_id)  
    id_base = len(gt_corr) / 2.0
    
    print('--- read ground truth poses---')
    gt_poses = readGTPose(submap_id)
    # print('length of gt_poses: %d' % len(gt_poses))
    
    print('--- load iss xyz and sift uv')
    iss_xyz = readISSXYZ(submap_id)
    sift_uv = readSIFTUV(submap_id)
    
    print('--- compute pose error---')   
    sum_error = 0.0
    num_of_valid_img = 0
    new_num = 0
    new_sum_error = 0
    # for each image, compute pose error    
    pred_poses = [None]*len(gt_poses)
    pred_poses_err = [None]*len(gt_poses)
    
    correct_patch_counter=np.zeros(len(gt_poses,))
    for i in range(len(pred_corr)):
        #print('***** submap image id: %d/%d' % (i, len(pred_corr)))
        if gt_corr[i] is None:
            continue
        
        num_of_valid_img += 1
        
        if i<id_base:
            #print('------ use left camera---')
            camera_matrix = camera_matrix_left
            T_ins_to_camera = T_ins_to_camera_left
            T_camera_to_cameraa = T_camera_to_cameraa_left
        elif i>=id_base and i<len(pred_corr):
            #print('------ use right camera---')
            camera_matrix = camera_matrix_right
            T_ins_to_camera = T_ins_to_camera_right
            T_camera_to_cameraa = T_camera_to_cameraa_right  
        else:
            print('Error image id!!!')
            exit
        
        sift_points = []
        iss_points = []
        for j in range(len(pred_corr[i].sift_id)):
            for k in range(top_k):
                sift_points.append(sift_uv[i][pred_corr[i].sift_id[j] - 1])
                iss_points.append(iss_xyz[pred_corr[i].iss_id[j][k] - 1])
                
        pred_poses[i] = getPnPSolution(sift_points,iss_points,camera_matrix, T_ins_to_camera, T_camera_to_cameraa)
        
        if pred_poses[i] is None:
            continue
        
        pred_poses_terror =np.linalg.norm(pred_poses[i][0:3] - gt_poses[i][0:3])
        gt_rot_matrix, _ = cv2.Rodrigues(np.array(gt_poses[i][3:6])*3.1415926536/180);
        pred_rot_matrix,_ = cv2.Rodrigues(pred_poses[i][3:6])
        rerror_vec,_ = cv2.Rodrigues(np.matmul(np.transpose(pred_rot_matrix), gt_rot_matrix))
        pred_poses_rerror = np.sqrt(np.sum(rerror_vec**2))
        
        if pred_poses_terror > 10:
            print i, pred_poses_terror
        
        if pred_poses_terror <= 10:
            new_sum_error += pred_poses_terror
            new_num += 1
        
        sum_error += pred_poses_terror
        pred_poses_err[i] = [pred_poses_terror, pred_poses_rerror]

    print ('average error: %.2f' % (sum_error/num_of_valid_img))
    print ('average error (err<=10): %.2f, %d/%d' % (new_sum_error/new_num, new_num, num_of_valid_img))
    return pred_poses, pred_poses_err, correct_patch_counter
    


    

if __name__ == '__main__':
    
    submap_id = 11
    
    pred_poses, pred_poses_err,correct_patch_counter = computeAccuracy(submap_id)
    
    