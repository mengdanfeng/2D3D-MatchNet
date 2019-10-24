close all
clear all
clc

% 
% date_time = '2014-06-26-09-53-12';
date_time = '2015-07-14-16-17-39';

loadGPSParams_1
% rgb_image_path = strcat(base_path, '/rgb_left/', date_time);
rgb_image_path = strcat('/media/mengdan/DATA1/mengdan/rgb_left/', date_time);

if ~exist(rgb_image_path,'dir')
    mkdir(rgb_image_path);
end

% convert images to rgb
% synchronize with ins
for i=1:2
    [camera_timestamps{i}] = removePoseWOData(camera_timestamps{i}, camera_dir{i}, 1);
    [camera_timestamps{i}] = synchronizeWithINS(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps{i});        
end

% get all camera global poses
fprintf('---Get camera global poses\n');
cam_global_poses = cell(2,1);
for i=1:2
    cam_global_poses{i} = cell(size(camera_timestamps{i},1),1);
    for chunk=1:camera_timestamps{i}(end,2)
        fprintf('  Processing chunk: %d camera %d poses\n', chunk, i)
        idx1 = size(find(camera_timestamps{i}(:,2) < chunk),1);
        idx2 = size(find(camera_timestamps{i}(:,2) <= chunk),1);
        poses = getGlobalPoses(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps{i}( camera_timestamps{i}(:,2) == chunk, 1)');    
        cam_global_poses{i}(idx1+1:idx2) = poses;    
    end
end

% remove cameras with small poses
[camera_timestamps, cam_global_poses] = ...
    removeSmallCameraPoses(camera_timestamps, cam_global_poses, camera_reading_angle, camera_reading_distance);

% plot trajectory
num_cameras = length(cam_global_poses{1});
cam_positions = zeros(num_cameras,2);

for i=1:num_cameras
    cam_positions(i,:) = cam_global_poses{1}{i}(1:2,4);
end

figure,
% plot(cam_positions(:,1),cam_positions(:,2),'b.');
plot(cam_positions(2000:8000,1),cam_positions(2000:8000,2),'b.');

% get rgb images
camera_timestamps = camera_timestamps{1};
rgb_images = cell(length(camera_timestamps), 1);
image_paths = cell(length(camera_timestamps),1);
for i=2000:8000
    disp(i)
    I = LoadImage(camera_dir{1}, camera_timestamps(i,1), LUT{1});
    if isempty(I)
        fprintf('  image does not exist. dir:%s, time: %f\n', camera_dir{1},camera_timestamps);
    end
    
    rgb_images{i} = I;
    image_paths{i} = sprintf('%s/%05d.png', rgb_image_path, i);
%     imwrite(I, file_path);
end

new_rgb_images = cell(6001,1);
new_image_paths = cell(6001,1);
new_rgb_images = rgb_images(2000:8000,1);
new_image_paths = image_paths(2000:8000,1);
 
% save rgb images
% cellfun(@imwrite, rgb_images, image_paths);
cellfun(@imwrite, new_rgb_images, new_image_paths);


function [camera_timestamps, cam_global_poses] = ...
    removeSmallCameraPoses(camera_timestamps, cam_global_poses, camera_reading_angle, camera_reading_distance)
% remove camera poses with small motion

% get relative transformation
cam_rel_poses_Norm = zeros(size(camera_timestamps{1},1), 2);

for i=2:size(camera_timestamps{1},1)
    T = cam_global_poses{1}{i-1} \ cam_global_poses{1}{i};
    [x1, y1, z1]= getEulerAngles(T(1:3,1:3));
    cam_rel_poses_Norm(i,:) = [norm([x1, y1, z1])*180/pi, norm(T(1:3,4))]; 
end

% remove camera poses with small movement
idx = find(cam_rel_poses_Norm(:,1) > camera_reading_angle | cam_rel_poses_Norm(:,2) > camera_reading_distance);
idx = [1; idx];
camera_timestamps{1} = camera_timestamps{1}(idx,:);
camera_timestamps{2} = camera_timestamps{2}(idx,:);
cam_global_poses{1} = cam_global_poses{1}(idx,1);
cam_global_poses{2} = cam_global_poses{2}(idx,1);
end