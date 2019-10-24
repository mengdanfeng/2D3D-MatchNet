close all
clear all
clc

% 
date_time = '2014-06-26-09-53-12';  % loc: low_id: 1000, high_id:8000
% date_time = '2015-07-14-16-17-39';  % map: low_id: 2500, high_id: 11000

%% location params
base_path = '/media/mengdan/data3/robotcar/grasshopper';
camera_base_dir = '/media/mengdan/DATA1/mengdan/datasets/RobotCar_Stereo/extracted_data';
camera = 'stereo';
camera_dir = strcat(camera_base_dir,'/',date_time,'/',camera,'/centre');

% models and extrinsics
camera_models_path = strcat(base_path, '/', 'robotcar-dataset-sdk/models/');
extrinsics_dir = strcat(base_path, '/', 'robotcar-dataset-sdk/extrinsics/');
ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);

% camera_timestamps
camera_timestamps=dlmread(strcat(camera_base_dir,'/',date_time,'/',camera, '.timestamps'));
camera_timestamps((camera_timestamps(:,2) == 0),:) = [];

%%%%%%%%%%%%%Camera Model%%%%%%%%%%%%%
[ fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel(camera_dir,camera_models_path);
camera_intrinsics=[fx,0,cx;0,fy,cy;0,0,1];

% camera param
camera_reading_angle= 10;
camera_reading_distance=0.02;

%% rgb_image_path = strcat(base_path, '/rgb_left/', date_time);
rgb_image_path = strcat('/media/mengdan/DATA1/mengdan/rgb_centre/', date_time);

if ~exist(rgb_image_path,'dir')
    mkdir(rgb_image_path);
end

% convert images to rgb
% synchronize with ins
camera_timestamps = removePoseWOData(camera_timestamps, camera_dir, 1);
camera_timestamps = synchronizeWithINS(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps);        


% get all camera global poses
fprintf('---Get camera global poses\n');

cam_global_poses = cell(size(camera_timestamps,1),1);
for chunk=1:camera_timestamps(end,2)
    fprintf('  Processing chunk: %d poses\n', chunk)
    idx1 = size(find(camera_timestamps(:,2) < chunk),1);
    idx2 = size(find(camera_timestamps(:,2) <= chunk),1);
    poses = getGlobalPoses(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps( camera_timestamps(:,2) == chunk, 1)');    
    cam_global_poses(idx1+1:idx2) = poses;    
end


% remove cameras with small poses
[camera_timestamps, cam_global_poses] = ...
    removeSmallCameraPoses(camera_timestamps, cam_global_poses, camera_reading_angle, camera_reading_distance);

% plot trajectory
num_cameras = length(cam_global_poses);
cam_positions = zeros(num_cameras,2);

for i=1:num_cameras
    cam_positions(i,:) = cam_global_poses{i}(1:2,4);
end

figure,
plot(cam_positions(:,1),cam_positions(:,2),'b.');
low_id = 1000;
high_id = 8000;
% low_id = 2500;
% high_id = 11000;

% file_name = 'pose_file';
% poses = zeros(high_id-low_id+1,7);
for i=1:(high_id-low_id+1)
    id = i+low_id-1;
    if mod(id,3) == 0
        poses(end+1,:) = [(cam_global_poses{id}(1:3,4))', rotm2quat(cam_global_poses{id}(1:3,1:3))];
    end
end
plot(poses(:,1),poses(:,2),'b.')
dlmwrite('loc_pose_file.txt', poses);
        
    

plot(cam_positions(low_id:high_id,1),cam_positions(low_id:high_id,2),'b.');

% get rgb images
im_number = high_id - low_id+1;
% rgb_images = cell(im_number, 1);
% image_paths = cell(im_number,1);
rgb_images = {};
image_paths = {};
image_id = 0;
for i=1:im_number
%     disp(i)
    id = i+low_id-1;
    if mod(id,3) == 0
        image_id = image_id + 1;
        disp(image_id)
        I = LoadImage(camera_dir, camera_timestamps(id,1), LUT);
        if isempty(I)
            fprintf('  image does not exist. dir:%s, time: %f\n', camera_dir,camera_timestamps);
        end
    
        rgb_images{end+1} = I;
        image_paths{end+1} = sprintf('%s/%05d.png', rgb_image_path, image_id);
    end
end

% save rgb images
cellfun(@imwrite, rgb_images, image_paths);


%% read poses from orb_slam2
orb_pose_file = '/home/mengdan/ORB_SLAM2/KeyFrameTrajectory.txt';
fid = fopen(orb_pose_file);
data = textscan(fid, '%f %f %f %f %f %f %f %f\n');
keyframe_id = data{1};
num_keyframe = length(keyframe_id);
keyframe_position = [data{2},data{3},data{4}];
keyframe_quaternion = [data{5},data{6},data{7}, data{8}];
keyframe_rotation = cell(num_keyframe,1);
for i=1:num_keyframe
    keyframe_rotation{i} = quat2rotm(keyframe_quaternion(i,:));
end

% get the ground truth poses
gt_position = zeros(num_keyframe,3);
gt_rotation = cell(num_keyframe,1);

pos_error = zeros(num_keyframe,1);
rot_error = zeros(num_keyframe,1);
% gt_rel_poses{1} = cam_global_poses{2500};
for i=1:length(keyframe_id)
    id = keyframe_id(i) + 2499;
%     gt_global_poses{i} = cam_global_poses{id}; 
    gt_rel_poses = cam_global_poses{keyframe_id(1) + 2499} \ cam_global_poses{id} ;
    gt_position(i,:) = gt_rel_poses(1:3,4);
    gt_rotation{i} = gt_rel_poses(1:3,1:3);
end

% computer error
pos_error = vecnorm((keyframe_position - gt_position),2, 2);
for i=1:num_keyframe
    rot_error(i) = norm(rotm2eul(keyframe_rotation{i}'*gt_rotation{i}));
end



function [camera_timestamps, cam_global_poses] = ...
    removeSmallCameraPoses(camera_timestamps, cam_global_poses, camera_reading_angle, camera_reading_distance)
% remove camera poses with small motion

% get relative transformation
cam_rel_poses_Norm = zeros(size(camera_timestamps,1), 2);

for i=2:size(camera_timestamps,1)
    T = cam_global_poses{i-1} \ cam_global_poses{i};
    [x1, y1, z1]= getEulerAngles(T(1:3,1:3));
    cam_rel_poses_Norm(i,:) = [norm([x1, y1, z1])*180/pi, norm(T(1:3,4))]; 
end

% remove camera poses with small movement
idx = find(cam_rel_poses_Norm(:,1) > camera_reading_angle | cam_rel_poses_Norm(:,2) > camera_reading_distance);
idx = [1; idx];
camera_timestamps = camera_timestamps(idx,:);
% camera_timestamps{2} = camera_timestamps{2}(idx,:);
cam_global_poses = cam_global_poses(idx,1);
% cam_global_poses{2} = cam_global_poses{2}(idx,1);
end