close all
clear all
clc

% date_time = '2015-07-14-16-17-39';

% date_time = '2015-04-24-08-15-07';
% date_time = '2015-08-13-16-02-58';

% date_time = '2014-06-26-09-53-12';
date_time = '2015-07-14-16-17-39';


loadGPSParams_1

% compute camera poses
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

% plot camera poses
num_cameras = length(cam_global_poses{1});
cam_positions = zeros(num_cameras,2);

for i=1:num_cameras
    cam_positions(i,:) = cam_global_poses{1}{i}(1:2,4);
end

fprintf('---Get all submaps for camera and laser\n');
submap_cover_distance = 60;
camera_submap_idx = get_camera_submap_indices(camera_timestamps{1}, ...
    cam_global_poses{1}, submap_cover_distance);

total_submap_num = length(unique(camera_submap_idx));
test_submap_num = floor(length(unique(camera_submap_idx)) *0.1);
train_submap_num = total_submap_num - test_submap_num;

% spatial extent
% max(cam_positions(:,1)) - min(cam_positions(:,1))
% max(cam_positions(:,2)) - min(cam_positions(:,2))

cam_positions_1 = [];
cam_positions_test = [];
k=0;
for i=1:num_cameras
    if camera_submap_idx(i) > train_submap_num
        if mod(i,10) == 1
            cam_positions_test(end+1,:) = cam_global_poses{1}{i}(1:2,4);
        end
        continue;
    end
    
    if mod(i,10)==1
%         k = k+1;
        cam_positions_1(end+1,:) = cam_global_poses{1}{i}(1:2,4);
    end
end

figure(),
plot(cam_positions_1(:,1), cam_positions_1(:,2),'g.');
hold on
plot(cam_positions_test(:,1), cam_positions_test(:,2),'b*');
title(date_time)

