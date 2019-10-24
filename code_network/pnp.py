import cv2
import numpy as np


points_2d = np.array([[406.62625+10, 418.23987],
	                  [451.86676, 248.99527],
	                  [742.97742, 370.27945+10],
	                  [852.77582, 570.57867],
	                  [543.78070, 425.92725],
	                  [670.12653, 392.74426]])

points_3d = np.array([[19.325596, -14.556233, -1.2872040],
	                  [19.615261, -14.401790, -6.6040850],
	                  [22.929752, -5.5222430, -2.7835140], 
	                  [26.957468, -5.5123048, -0.26691201], 
	                  [31.638401, -31.393797, -4.1175189],
	                  [31.371483, -17.278540, -5.7012320]])

points_2d = np.expand_dims(points_2d, axis=1)
points_3d = np.expand_dims(points_3d, axis=1)


camera_matrix = np.array([[400, 0  , 500.107605000000], 
                          [0  , 400, 511.461426000000],
                          [0  , 0  , 1]])

pose = cv2.solvePnPRansac(points_3d[0:4, :], points_2d[0:4, :], camera_matrix, None, None, None, False, cv2.SOLVEPNP_EPNP)
#pose = cv2.solvePnPRansac(points_3d[0:4, :], points_2d[0:4, :], camera_matrix, None, flags=cv2.SOLVEPNP_EPNP)

print ('solvePnPRansac pose:')
print(pose[0])
print(np.transpose(pose[1]))
print(np.transpose(pose[2]))
print(np.transpose(pose[3]))

rot_matrix, _ = cv2.Rodrigues(pose[1])
T_ins1_to_camera = np.zeros([4, 4])
T_ins1_to_camera[0:3, 0:3] = rot_matrix
T_ins1_to_camera[0:3, 3] = np.squeeze(pose[2])
T_ins1_to_camera[3, 3] = 1.0

print (T_ins1_to_camera)
print ('----------------------------')

T_ins_to_camera = np.array([[0.313485785451577, -0.941953894385083,  0.120206169444769, -0.612518528650024],
						   [0.929455016766250,  0.278437884954273, -0.242044863672503, -0.238809567784806],
						   [0.194525150373564,  0.187603851439589,  0.962790091763088,  1.04417692175197 ],
						   [0, 0, 0, 1]])

#T_camera_to_cameraa = np.array([[0, 0, 1, 0], 
#	                            [1, 0, 0, 0],
#	                            [0, 1, 0, 0], 
#	                            [0, 0, 0, 1]])

T_camera_to_cameraa = np.array([[0, 1, 0, 0], 
	                            [0, 0, 1, 0],
	                            [1, 0, 0, 0], 
	                            [0, 0, 0, 1]])

T = np.matmul(np.linalg.inv(T_ins1_to_camera), np.matmul(T_camera_to_cameraa, T_ins_to_camera))
rvec,_ = cv2.Rodrigues(T[0:3,0:3])


print ('estimated pose:')
print (T)
print ('------------------------------')

T_gt = np.array([[0.998951285616171, -0.0446715377459037, -0.0100390577852267,	17.7920392949885],
                 [0.0447229370438996, 0.998987140410078, 0.00495501740277965, -0.197778748299623],
                 [0.00980754138233525, -0.00539879715406600, 0.999937330596935, 0.236492914339472],
                 [0, 0,	0, 1]])
rvec_gt,_ = cv2.Rodrigues(T_gt[0:3,0:3])
print ('---- estimate rvec')
print rvec
print rvec_gt
print np.linalg.norm(rvec_gt - rvec)

# points3d_homo = np.ones([4, 6])
# points3d_homo[0:3, :] = points_3d.transpose()
# project_3d = np.matmul(T_camera_to_cameraa, np.matmul(T_ins_to_camera, np.matmul(np.linalg.inv(T_gt), points3d_homo)))
# homo_2d = np.matmul(camera_matrix, project_3d[0:3, :])
# points_2d = homo_2d[0 : 2, :] / homo_2d[2, :]
# print (points_2d)
# print ('------------------------')

# pose_gt = np.matmul(T_camera_to_cameraa, np.matmul(T_ins_to_camera, np.linalg.inv(T_gt)))
# print (pose_gt)
# print (T_ins1_to_camera)
