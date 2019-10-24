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

import numpy as np
import os
import sys
import cv2
from scipy.spatial import distance
import math

# --------- Parameters ------------
invaild_pairs_threshold = 4

top_k = 5

date_time =  '2015-02-13-09-16-26'

gt_correspondence_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_corr_gt/' + date_time
pred_correspondence_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_correspondences_64/' + date_time
gt_pose_root = '/media/mengdan/data3/robotcar/grasshopper/cam_global_poses/' + date_time

sift_iss_idx_value_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_index_value/'  + date_time

pred_result_root = '/media/mengdan/data3/robotcar/grasshopper/results/' + date_time

if not os.path.exists(pred_result_root):
    os.makedirs(pred_result_root)
    


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

# T_cameraa_to_camera/G_camera_image
T_cameraa_to_camera_left = np.array([[0, 0, 1, 0], 
	                            [1, 0, 0, 0],
	                            [0, 1, 0, 0], 
	                            [0, 0, 0, 1]])

T_cameraa_to_camera_right = np.array([[0,	0,	1,	0],
                                     [1,	0,	0,	0],
                                     [0,	1,	0,	0],
                                     [0,	0,	0,	1]])

T_camera_to_cameraa_left = np.linalg.inv(T_cameraa_to_camera_left)
T_camera_to_cameraa_right = np.linalg.inv(T_cameraa_to_camera_right)


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
    # pose: pose[0]: True/False, pose[1]:rvec, pose[2]:tvec, pose[3]:inlier_ids
    points_2d = np.expand_dims(points_2d, axis=1)
    points_3d = np.expand_dims(points_3d, axis=1)
    pose = cv2.solvePnPRansac(points_3d, points_2d, camera_matrix, None, None, None, False, iterationsCount=5000, flags=cv2.SOLVEPNP_EPNP)
    
    if pose[0] is False or np.isnan(pose[2]).any() or np.isnan(pose[1]).any():
        return None, None, None, None, None
    
#    if np.isnan(pose[2]).any():
#        print pose[0]

    # ins1 -> camera (origin ins 1's pose in camera coordinate)
    rot_matrix,_ = cv2.Rodrigues(pose[1])
    
    T_ins1_to_camera = np.zeros([4, 4])
    T_ins1_to_camera[0:3, 0:3] = rot_matrix
    T_ins1_to_camera[0:3, 3] = np.squeeze(pose[2])
    T_ins1_to_camera[3, 3] = 1.0

    T = np.matmul(np.linalg.inv(T_ins1_to_camera), np.matmul(T_camera_to_cameraa, T_ins_to_camera))
    
    # 4x4 matrix to rvec, tvec
    pred_pose_tvec = T[0:3,3]
    pred_pose_rot_mat = T[0:3,0:3]
    
    # inliers
    inlier_num = pose[3].shape[0]
    inlier_points_2d = points_2d[pose[3]]
    inlier_points_3d = points_3d[pose[3]]
    
    inlier_points_2d = np.array(inlier_points_2d).flatten().tolist()
    inlier_points_3d = np.array(inlier_points_3d).flatten().tolist()
#    print ('**** inlier points*****')
#    print inlier_points_2d
#    print inlier_points_3d
    return pred_pose_tvec, pred_pose_rot_mat, inlier_num, inlier_points_2d, inlier_points_3d


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])                          
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-3
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        roll = math.atan2(R[2,1] , R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw = math.atan2(R[1,0], R[0,0])
    else :
        roll = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw = 0
 
    return np.array([roll, pitch, yaw])
    
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
    num_of_correct_img = 0
    num_of_valid_img = 0
    correct_num = 0
    correct_pose_sum_terror = 0.0
    correct_pose_sum_rerror = 0.0
    # for each image, compute pose error    
    pred_poses_trans = [None]*len(gt_poses)
    pred_poses_rot = [None]*len(gt_poses)
    pred_poses_err = [None]*len(gt_poses)
    pred_poses_inlier_num = np.zeros(len(gt_poses))
    pred_poses_inlier_points_2d = [None]*len(gt_poses)
    pred_poses_inlier_points_3d = [None]*len(gt_poses)
    total_inlier_num = 0
    
#    correct_patch_counter=np.zeros(len(gt_poses,))
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
        
        num_of_correct_pairs = 0
        sift_points = []
        iss_points = []
        for j in range(len(gt_corr[i].sift_id)):
            if gt_corr[i].sift_id[j] > len(pred_corr[i].sift_id):
                continue
            
            top_k_iss_id = pred_corr[i].iss_id[gt_corr[i].sift_id[j] - 1]
            if gt_corr[i].iss_id[j] in top_k_iss_id:
                # insert sift and iss points
                sift_points.append(sift_uv[i][gt_corr[i].sift_id[j] - 1])
                iss_points.append(iss_xyz[gt_corr[i].iss_id[j] - 1])
                
                num_of_correct_pairs += 1
                
        #correct_patch_counter[i]=num_of_correct_pairs
        
        if num_of_correct_pairs > 3:
            
            num_of_correct_img += 1          
            pred_poses_trans[i], pred_poses_rot[i] , pred_poses_inlier_num[i], pred_poses_inlier_points_2d[i], pred_poses_inlier_points_3d[i] = \
                getPnPSolution(sift_points,iss_points,camera_matrix, T_ins_to_camera, T_camera_to_cameraa) 
            
            if pred_poses_trans[i] is None:
                continue
            
            total_inlier_num += pred_poses_inlier_num[i]
            #print total_inlier_num
            
            # translation error and rotation error
            pred_poses_terror =np.linalg.norm(pred_poses_trans[i][0:3] - gt_poses[i][0:3])
            
            gt_rot_matrix = np.reshape(gt_poses[i][3:],[3,3])
            pred_rot_matrix = pred_poses_rot[i]
#            print ('***** check  rotation matrix*******')
#            print pred_rot_matrix, pred_poses_trans[i]
            rerror_vec = rotationMatrixToEulerAngles(np.matmul(np.transpose(pred_rot_matrix), gt_rot_matrix))
            pred_poses_rerror = np.linalg.norm(rerror_vec)
            #print pred_poses_rerror
            
            terror_val = 10
            rerror_val = 0.8
            
#            if pred_poses_terror > terror_val or pred_poses_rerror > rerror_val:
#                print i, pred_poses_terror
            
            if pred_poses_terror <= terror_val and pred_poses_rerror <= rerror_val:
                correct_pose_sum_terror += pred_poses_terror
                correct_pose_sum_rerror += pred_poses_rerror
                correct_num += 1
            
            sum_error += pred_poses_terror
            pred_poses_err[i] = [pred_poses_terror, pred_poses_rerror]            
    
    # write to file
#    with open(pred_result_root+'/'+str('%03d' % submap_id) + '.txt', 'w') as file:
#        # write the first line
#        file.write('submap_id, correct_imgs, valid_imgs, total_inliers, avg_terror, avg_rerror\n')
##        print submap_id
##        print correct_num
##        print num_of_valid_img
##        print int(total_inlier_num)
##        print correct_pose_sum_terror/correct_num
##        print correct_pose_sum_rerror/correct_num * 180/math.pi
#        line = [submap_id,correct_num,num_of_valid_img, int(total_inlier_num), \
#                round(correct_pose_sum_terror/correct_num, 3), \
#                round(correct_pose_sum_rerror/correct_num * 180/math.pi, 3)]
#        file.write(str(line).strip('[]') +'\n')
#        
#        
#        # write the rest
#        file.write('\ncam_id, img_id, pred_err_t, pred_err_r, pred_t, pred_r, inlier_num, sift_uv, iss_xyz\n')
#        for i in range(len(pred_corr)):
#            # for valid poses, record all information
#            #print i
#            cam_id = i // (len(pred_corr)/2)
#            img_id = i - (cam_id * (len(pred_corr)/2))
#            
#            if pred_poses_trans[i] is None:
#                line = [cam_id+1, img_id+1, pred_poses_trans[i]]
#                #print line
#                file.write(str(line).strip('[]')+ '\n')
#                continue
#            
#            pred_err = pred_poses_err[i][0:2]       # 2-element list, [terror,rerror]
#            pred_trans = pred_poses_trans[i].tolist()    # 3-element list, tvec
#            pred_rot = pred_poses_rot[i].flatten().tolist() # 9-element list, rmat
#            inlier_num = pred_poses_inlier_num[i]
#            sift_uv = pred_poses_inlier_points_2d[i]            
#            iss_xyz= pred_poses_inlier_points_3d[i] 
#            
#            line = [cam_id+1, img_id+1] + pred_err + pred_trans + pred_rot + [inlier_num] + sift_uv + iss_xyz
#            file.write(str(line).strip('[]') +'\n')
            
    print ('number of correct images: %d / %d, %d' % 
           (num_of_correct_img, num_of_valid_img, len(pred_corr)))
    print ('top %d accuracy = %.2f%%' % (len(top_k_iss_id), float(num_of_correct_img) / num_of_valid_img * 100.0))
    print ('average error: %.2f' % (sum_error/num_of_valid_img))
    print ('average error (err<=%d): %.2fm, %.2fdeg, %d/%d' % (terror_val,correct_pose_sum_terror/correct_num, correct_pose_sum_rerror/correct_num * 180/math.pi, correct_num, num_of_valid_img))
    print ('total inlier num: %d' % total_inlier_num) 
    #return pred_poses_trans, pred_poses_err  
    
    result = [correct_num,num_of_valid_img, int(total_inlier_num), correct_pose_sum_terror, \
             (correct_pose_sum_rerror * 180/math.pi)]
    
    return result
    

if __name__ == '__main__':
    
#    submap_id = 82
#    result = computeAccuracy(submap_id)
    
    total_test_frames = 0
    successful_frames = 0
    total_inliers = 0
    total_rerror = 0.0
    total_terror = 0.0    
    
    submap_ids = range(116,122)
    #submap_ids = [116,117]
    
    for i in submap_ids:
        print ('\n******* progress %d**********' % (i))
        submap_id = i
        result = computeAccuracy(submap_id)
        successful_frames += result[0]
        total_test_frames += result[1]
        total_inliers += result[2]
        total_terror += result[3]
        total_rerror += result[4]
    
    mean_terror = total_terror/successful_frames
    mean_rerror = total_rerror/successful_frames
    print successful_frames, total_test_frames
    print successful_frames, total_inliers
    print mean_terror, mean_rerror
    
        
        
    

                
    
    
    