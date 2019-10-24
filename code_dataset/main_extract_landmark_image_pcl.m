clc
clear all
close all

%% Load pre-defined params
load_retrieval_params

%% step1: assign reference run
ref_date_time = '2014-12-02-15-30-08';

% Get related params from given date_time
[laser_dir, camera_dir, laser_timestamps, camera_timestamps, camera_intrinsics, G_camera_image, LUT] = ...
    getParamsFromDatetime(ref_date_time);

% get reference landmark images
[ref_landmark_image_ids, ref_landmark_global_poses] = ...
    extractLandmarkImages(camera_timestamps, cam);


%% step2: choose a good reference run and extract landmark image frames
% choose reference run: 11/50, 2014-12-02-15-30-08
ref_date_time = '2014-12-02-15-30-08';
image_dir = sprintf('%s/%s/%s', camera_base_path, ref_date_time, image_type);


[ref_landmark_image_ids, ref_landmark_global_poses] = ...
    extractLandmarkImages(camera_timestamps, cam)

for i=1:length(date_times)
    date_time = date_times(i).name;
    fprintf('Processing: %d/%d, %s\n', i, length(date_times), date_time);
    image_files = dir([sprintf('%s/%s/%s/', camera_base_path, date_time, image_type) '*.png']);
    ins_file = sprintf('%s/%s/%s', gps_base_path, date_time, gps_type);
    
    pose_timestamps = zeros(length(image_files),1);
    for j=1:length(image_files)
        [~,pose_timestamp, ~] = (fileparts(image_files(j).name));
        pose_timestamps(j) = uint64(str2double(pose_timestamp));
    end

    poses = getGlobalPoses1(ins_file, pose_timestamps');
    saveas(gcf, sprintf('%s/pose_figures/%s.png', data_base_path, date_time));
    close all
end
% 
% 
% 
% [landmark_frame_ids, landmark_timestamps, landmark_laser_idx, landmark_global_poses] = ...
%     extractLandmarkImages(camera_timestamps, camera_laser_idx, cam_global_poses, landmark_dist);
% 
% % processing
% for i=1:length(folders)
%     date_time = folders(i).name
%     %fprintf('** process ')
%     main(date_time, camera_base_path, laser_base_path, data_base_path);
% end
% 
% %% main function  
% 
% 
% 
% 
% % build pointcloud for each landmark image
% [landmark_image, landmark_pcl, is_valid] = ...
%     extractLandmarkPCL(landmark_frame_ids, landmark_laser_idx);
% 
% %% aligh other runs to reference run
