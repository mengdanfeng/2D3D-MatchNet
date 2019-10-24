clc
clear all
close all

%% step1: load and set params
% list all runs
folders = dir('/media/mengdan/data3/robotcar/grasshopper/sift_patch');
folders = folders(3:end);

% base path
%camera_base_path = '/media/mengdan/data3/robotcar/bumblebee';
camera_base_path = '/media/mengdan/DATA1/mengdan/datasets/RobotCar_Stereo';
laser_base_path = '/media/mengdan/data3/robotcar/grasshopper';
data_base_path = '/media/mengdan/data3/robotcar/bumblebee';

for i=2:length(folders)
    date_time = folders(i).name;
    fprintf('Process %d/%d: %s\n', i, length(folders), date_time);
    main(date_time, camera_base_path, laser_base_path, data_base_path);
end    


%date_time = '2014-06-26-09-53-12';

function main(date_time, camera_base_path, laser_base_path, data_base_path)

% lidar 
laser = 'lms_front';
laser_dir = strcat(laser_base_path, '/', laser, '/', date_time, '/', laser);

% camera
camera = 'stereo';
camera_dir = strcat(camera_base_path, '/', 'extracted_data', '/', date_time, '/', camera, '/', 'centre');

camera_models_path = strcat(laser_base_path, '/', 'robotcar-dataset-sdk/models/');


%%%%%% load extrinsics %%%%%%%%%%%%%%
extrinsics_dir = strcat(laser_base_path, '/', 'robotcar-dataset-sdk/extrinsics/');
laser_extrinisics = dlmread([extrinsics_dir 'lms_front.txt']);
ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);
camera_extrinsics = dlmread(strcat(extrinsics_dir,camera,'.txt'));

G_ins_laser = SE3MatrixFromComponents(ins_extrinsics) \ SE3MatrixFromComponents(laser_extrinisics);
G_camera_ins = SE3MatrixFromComponents(camera_extrinsics) * SE3MatrixFromComponents(ins_extrinsics);


%%%%%%% Timestamps %%%%%%%%%%%%%
laser_timestamps = dlmread(strcat(laser_dir,'.timestamps'));
camera_timestamps=dlmread(strcat(camera_base_path, '/', 'extracted_data', '/', date_time, '/', camera, '.timestamps'));
camera_timestamps((camera_timestamps(:,2) == 0),:) = [];


%%%%%%%%%%%%%Camera Model%%%%%%%%%%%%%
[ fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel(camera_dir,camera_models_path);
camera_intrinsics=[fx,0,cx;0,fy,cy;0,0,1];

% camera param
camera_reading_angle= 10;
camera_reading_distance=0.02;
%submap_cover_distance=60.0;


%% ###################### step2: get camera and lidar global pose
%% step2-1 synchronize timestamps
camera_timestamps = removePoseWOData(camera_timestamps, camera_dir, 1);
camera_timestamps = synchronizeWithINS(strcat(camera_base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps);        

laser_timestamps = removePoseWOData(laser_timestamps, laser_dir, 2);
laser_timestamps = synchronizeWithINS(strcat(camera_base_path, '/gps/', date_time, '/gps/ins.csv'), laser_timestamps);

[camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps2(camera_timestamps, laser_timestamps);

%% step2-2: get global poses for all camera 
% get all camera global poses
fprintf('----- Get camera global poses -----\n');
cam_global_poses = cell(size(camera_timestamps,1),1);

for chunk=1:camera_timestamps(end,2)
    fprintf('  Processing chunk: %d camera poses\n', chunk)
    idx1 = size(find(camera_timestamps(:,2) < chunk),1);
    idx2 = size(find(camera_timestamps(:,2) <= chunk),1);
    poses = getGlobalPoses(strcat(camera_base_path, '/gps/', date_time, '/gps/ins.csv'), ...
        camera_timestamps( camera_timestamps(:,2) == chunk, 1)');    
    cam_global_poses(idx1+1:idx2) = poses;    
end

% get global poses for all laser 
fprintf('----- Get laser global poses -----\n');
laser_global_poses = cell(size(laser_timestamps,1),1);
for chunk=1:laser_timestamps(end,2)
    fprintf('  Processing chunk: %d laser poses\n', chunk)
    idx1 = size(find(laser_timestamps(:,2) < chunk),1);
    idx2 = size(find(laser_timestamps(:,2) <= chunk),1);
    poses=getGlobalPoses(strcat(camera_base_path, '/gps/', date_time, '/gps/ins.csv'), ...
        laser_timestamps( laser_timestamps(:,2) == chunk, 1)'); 
    laser_global_poses(idx1+1:idx2) = poses;    
end

%% step2-3: filter camera poses
% remove cameras without camera and/or laser INS readings 
 [camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeCamPoseWOINS_2(camera_timestamps, camera_laser_idx, cam_global_poses, laser_global_poses);

% remove camera poses with small relative movement
[camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeSmallCamPoses_2(camera_timestamps, camera_laser_idx, cam_global_poses,...
    camera_reading_angle, camera_reading_distance);

%% ################### step3: establish relation between image and pointcloud
fprintf('----- Get image and pcl -----\n')
rgb_image = 'rgb_image';
image_pcl = 'image_pcl';
image_dir =  strcat(data_base_path, '/', rgb_image, '/', date_time);
pcl_dir = strcat(data_base_path, '/', image_pcl, '/', date_time);
if ~exist(image_dir, 'dir')
    mkdir(image_dir);
end

if ~exist(pcl_dir, 'dir')
    mkdir(pcl_dir);
end

image_dist = 15;
%image_interval = 16;
laser_frames_number = 50;

% extract image every 15m


image_num = floor(length(camera_timestamps)/double(image_interval));
images = cell(image_num,1);
pointclouds = cell(image_num,1);
image_filenames = cell(image_num,1);
pcl_filenames = cell(image_num,1);
for i=1:length(camera_timestamps)
    % extract image every K frame
    if mod(i,image_interval) == 0      
        fprintf('     progress: %d/%d\n', i, length(camera_timestamps));
        % build nearby +-N frames pointcloud        
        [image, pointcloud, is_valid] = buildPCL(i, camera_laser_idx, laser_frames_number, ...
            laser_timestamps, laser_global_poses, laser_dir, G_ins_laser, ...
            camera_timestamps, camera_intrinsics, cam_global_poses, ...
            G_camera_image, G_camera_ins, camera_dir, LUT);       
        
        % valid image and correspoinding pcl
        if is_valid
            idx = i/image_interval;
            images{idx} = image;
            pointclouds{idx} = pointCloud(pointcloud');
            image_filenames{idx} = [image_dir '/' num2str(camera_timestamps(i,1)) '.png'];
            pcl_filenames{idx} = [pcl_dir '/'  num2str(camera_timestamps(i,1)) '.pcd'];
        end
    end  
end

% remove empty data
images = images(~cellfun(@isempty, images));
image_filenames = image_filenames(~cellfun(@isempty, image_filenames));
pointclouds = pointclouds(~cellfun(@isempty, pointclouds));
pcl_filenames = pcl_filenames(~cellfun(@isempty, pcl_filenames));

% check consistency
if ~(length(images) == length(image_filenames) && length(images) == length(pointclouds) ...
        && length(images) == length(pcl_filenames))
    fprintf('  Data inconsistency!\n');
    return;
end

% save image and pcl
cellfun(@imwrite, images, image_filenames);
cellfun(@pcwrite, pointclouds, pcl_filenames);

end

