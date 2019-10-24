%% plot orb slam poses
% file = '/home/mengdan/ORB_SLAM2/KeyFrameTrajectory.txt';
% % file = '/home/mengdan/ORB_SLAM2/robotcar_results/section4/poses1.txt';
% fid = fopen(file);
% 
% data = textscan(fid, '%f %f %f %f %f %f %f %f');
% 
% xyz = [data{2},data{3},data{4}];
% plot(xyz(:,1),xyz(:,2))

%% plot ground truth point cloud
% date_time = '2014-06-26-09-53-12';
date_time = '2015-07-14-16-17-39';

loadGPSParams_1

for i=1:2
    [camera_timestamps{i}] = removePoseWOData(camera_timestamps{i}, camera_dir{i}, 1);
    [camera_timestamps{i}] = synchronizeWithINS(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps{i});        
end
laser_timestamps = removePoseWOData(laser_timestamps, laser_dir, 2);
laser_timestamps = synchronizeWithINS(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), laser_timestamps);

% synchronize
[camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps1(camera_timestamps, laser_timestamps);

% build pcl
ins_file = strcat(base_path, '/gps/', date_time, '/gps/ins.csv');
start_timestamp = laser_timestamps(camera_laser_idx(2600),1);
end_timestamp = laser_timestamps(camera_laser_idx(5900),1);
[pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, extrinsics_dir, ...
    start_timestamp, end_timestamp);

% visualize
ptCloud = pointCloud(pointcloud');
pcl = pcdownsample(ptCloud,'gridAverage',0.5);
figure,pcshow(pcl)
hold on
grid off
axis off

%% plot tra


