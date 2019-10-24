close all
clear all
clc

%%
% load params
loadParams

%% ########## step1: sychronize timestamps, thus both camera have the same timestamps, each camera has nearest laser id.
fprintf('Step1\n');
[camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps(camera_timestamps, laser_timestamps);

% figure,
% plot(laser_timestamps(:,1), 'r.')
% hold on
% plot(camera_laser_idx, camera_timestamps{1}(:,1), 'b.')
% plot(camera_laser_idx, camera_timestamps{2}(:,1), 'g.')
% title('synchronized camera and lidar timestamps')


%% ########## step2: get relative poses for each submap #########
fprintf('Step2\n');
% 2-1 for each camera_timestamp, get relative pose within each submap
camera_submap_idx = cell(2,1);
camera_relative_poses = cell(2,1);
camera_relative_poses_Norm = cell(2,1);
for i=1:2
    fprintf('  Processing camera %d poses\n', i);
    [cam_timestamps, submap_idx, cam_rel_poses, cam_rel_poses_Norm] =  GetCamSubmapIDAndRelPose(camera_timestamps{i}, vo_file, submapParams);
    camera_timestamps{i} = cam_timestamps;
    camera_submap_idx{i} = submap_idx;
    camera_relative_poses{i} = cam_rel_poses;
    camera_relative_poses_Norm{i} = cam_rel_poses_Norm;
end        


%% ######### step3: establish camera laser relation #############
fprintf('Step3\n');
% 3-1 pick those laser_timestamps correspond to camera_timestamps, 
%     remove the rest
[~, first_laser_idx] = min(abs(camera_timestamps{1}(1,1) - laser_timestamps(:,1)));
[~, last_laser_idx] = min(abs(camera_timestamps{1}(end,1) - laser_timestamps(:,1)));
laser_timestamps = laser_timestamps(first_laser_idx:last_laser_idx,:);
fprintf('  first laser_timestamps - camera_timestamps (should <=0): %f\n', (laser_timestamps(1,1)-camera_timestamps{1}(1,1)));
fprintf('  last laser_timestamps - camera_timestamps (should >=0): %f\n', (laser_timestamps(end,1)-camera_timestamps{1}(end,1)));

% remove small motion of laser scans (too slow, too many laser scans)
% [laser_timestamps, small_laser_idx] = removeSmallLaserMotion(laser_timestamps, vo_file, submapParams);

% 3-2 for each camera_timestamp, find the closest lidar id
camera_laser_idx = zeros(size(camera_timestamps{1},1),1);
for i=1:size(camera_timestamps{1},1)
   [val, idx] = min(abs(camera_timestamps{1}(i,1) - laser_timestamps(:,1)));
   camera_laser_idx(i,1) = idx;
end

% 3-3 for each submap, get the laser scan idx 
camera_submap_idx = camera_submap_idx{1};
num_submap = length(unique(camera_submap_idx));
%submap_laser_idx = zeros(size(laser_timestamps,1),1);
submap_laser_idx = cell(num_submap,1);
%first_laser_idx = camera_laser_idx(1);

for i=1:num_submap
    cam_idx = find(camera_submap_idx==i);
    first_laser_idx = camera_laser_idx(cam_idx(1)); 
    last_laser_idx = camera_laser_idx(cam_idx(end));    
    submap_laser_idx{i} = (first_laser_idx:last_laser_idx);
    %submap_laser_idx(first_laser_idx:last_laser_idx) = i;
    %first_laser_idx = last_laser_idx+1;
end


%% ########## step4: for each submap, remove ground plane, visualize pointcloud
fprintf('Step4\n');
% 4-1 get each submap, remove ground plane
submap_pointclouds = cell(num_submap,1);
for submapID = 1:num_submap
     submap_timestamps = laser_timestamps(submap_laser_idx{submapID},1);
     submap_pointcloud = BuildPCLVO(submap_timestamps, laser_dir, G_ins_laser, vo_file);
     submap_pointclouds{submapID} = submap_pointcloud;  
     
     % remove ground plane
     [normal, in_plane, out_plane]=pcfitplane(pointCloud(submap_pointcloud'),0.6);
     out_of_plane= submap_pointcloud(:,out_plane);
     
     % save pcl as pcd
     pcd_path = sprintf('%s/%03d.pcd', pcd_dir, submapID); 
     pcd_fig_path = sprintf('%s/%03d.png',pcd_dir, submapID);
     writePCLtoPCD(out_of_plane, pcd_path, pcd_fig_path);
end    
save('submap_pointclouds_vo50.mat','submap_pointclouds');

%% ****** STOP HERE for ISS keypoints detection, using c++
% ******* STOP, wait for keypoints *********
% ****** STOP HERE for ISS keypoints detection, using c++

%%
% 4-2 for each submap, load and visualize ISS keypoints
pcd_dir = strcat(pcd_dir, '/');
pcd_files = dir([pcd_dir '*.pcd']);
iss_files = dir([keypoints_dir '/' '*.pcd']);
submap_keypoints = cell(num_submap,1);
for submapID = 1:num_submap
    % submap pointcloud
    % pointcloud = pcread(strcat(pcd_dir, pcd_files(submapID).name));
    % green color
%     pointcloud_color = uint8(zeros(pointcloud.Count,3));
%     pointcloud_color(:,1) = 0;
%     pointcloud_color(:,2) = 255;
%     pointcloud_color(:,3) = 0;
%     pointcloud.Color = pointcloud_color;      
    
    % submap keypoints
    key_points = pcread(strcat(keypoints_dir, '/', iss_files(submapID).name));
    submap_keypoints{i} = key_points;
    % red color
%     key_points_color = uint8(zeros(key_points.Count,3));
%     key_points_color(:,1)=255;
%     key_points_color(:,2)=0;
%     key_points_color(:,3)=51;     
%     key_points.Color=key_points_color;
%     
%     figure, pcshow(pointcloud);
%     hold on
%     pcshow(key_points, 'MarkerSize',15);
    %saveas(gcf, sprintf('%s/%03d.png',submap_keypoints_dir,submapID));
end  
    

%% ######### step5: for each submap, project ISS points to each image ##############
fprintf('Step5\n');
% 5-1 for each submap, compute sift features for each image
submap_info = cell(num_submap,1);
for j=1:num_submap    
    fprintf('progress %d/%d\n',j,num_submap);
    cam_idx = find(camera_submap_idx == j);    
    % images{1=left/2=right}{img_id} = image
    % image_features{1=left/2=right}{img_id} = sift_keypoint
    images=cell(2,1);
    image_features=cell(2,1);
    cam_global_poses = cell(2,1);
    
    reproj_point_img_idx_list=cell(2,1);
    reproj_point_img_uv_list=cell(2,1);
    point_to_uv_correspondences=cell(2,1);
    
    % compute SIFT
    fprintf('  compute SIFT\n');
    for i=1:2
        %fprintf('  extracting sift features of camera %d ...\n', i);       
        [images{i}, image_features{i}] = ...
            computeImageFeatures(cam_idx, camera_dir{i}, camera_timestamps{i}, LUT{i});
    end
    
    % compute global camera poses within each submap
    fprintf('  compute global poses\n');
    for i=1:2  
        relative_poses = RelativeToAbsolutePoses(vo_file, reshape(camera_timestamps{i}(cam_idx),1,[]), camera_timestamps{i}(cam_idx(1)));
        cam_global_poses{i} = relative_poses;
    end     
        
    % project ISS keypoints to image
    fprintf('  project iss keypoints to image\n');
    for i=1:2
        [reproj_point_img_idx_list{i}, reproj_point_img_uv_list{i}] = ProjIss2Img(images{i}, cam_idx, submap_pointclouds{j}, ...
            cam_global_poses{i}, G_camera_image{i}, G_camera_ins{i}, camera_intrinsics{i},j,pcl_proj_dir,i);
    end
       
    % search for correspondences
    fprintf('  search for 2d-3d correspondences\n');
    for i=1:2
        point_to_uv_correspondences{i} = ...
            getCorrespondences(images{i}, submap_pointclouds{j}, cam_idx, image_features{i}, ...
            reproj_point_img_uv_list{i}, reproj_point_img_idx_list{i}, corrThreshold);
    end          
    submap_info{j}.cam_idx = cam_idx;
    submap_info{j}.reproj_point_img_idx_list = reproj_point_img_idx_list;
    submap_info{j}.reproj_point_img_uv_list = reproj_point_img_uv_list;
    submap_info{j}.point_to_uv_correspondences = point_to_uv_correspondences;
end
save('submap_info_100m.mat','submap_info');
% % visualize 2d path
% cam_poses = cam_global_poses{1};
% cam_pos = zeros(2, length(cam_poses));
% for i=1:length(cam_poses)
%     xy = cam_poses{i}(1:2,4);
%     cam_pos(:,i) = xy;
% end



function [camera_timestamps, submap_idx, cam_relative_poses, cam_relative_poses_Norm] =  GetCamSubmapIDAndRelPose(camera_timestamps, vo_file, submap_params)
%% Target: get submap indices for all the camera_timestamps
% Input: 
%   camera_timestamps: all the camera_timestamps
%   vo_file: relative poses (x,y,z, alpha, beta, gamma) between each two
%            timestamps
%   submap_cover_distance: the length of each divided submap
% Output:
%   submap_idx: the submap id for each camera_timestamp

%% 
submap_idx = [];
submap_idx(1) = 1;
sum_dist = 0;
submap_counter = 1;
cam_relative_poses = {};
cam_relative_poses{1} = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
cam_relative_poses_Norm = [];
cam_relative_poses_Norm(1,:) = zeros(1,2);
relative_poses_counter = 1;
small_movement_id = [];

origin_id = 1;
for i=2:size(camera_timestamps,1)
    if(mod(i,100)==1)
        fprintf('    progress: %d/%d\n', i, size(camera_timestamps,1));
    end
    % 4x4 relative poses    
    %fprintf('origin_id: %d, current_id: %d\n', origin_id, i);
    origin_timestamp = camera_timestamps(origin_id,1);
    current_timestamp = camera_timestamps(i,1);
    cam_rel_pose = RelativeToAbsolutePoses(vo_file,current_timestamp,origin_timestamp); 
    cam_rel_pose = cam_rel_pose{1};
    
    % 3x3 rotation matrix to 1x3 euler angle vector
    [alpha, belta, gamma] = getEulerAngles(cam_rel_pose(1:3,1:3));
    
    % relative distance for both angle and position
    cam_rel_pose_Norm = [norm([alpha, belta, gamma]), norm(cam_rel_pose(1:3,4))];
    %fprintf("pos: %03f, angle: %03f, \n",cam_rel_pose_Norm(1,1), cam_rel_pose_Norm(1,2));
    
    % remove camera poses with small movement
    if (cam_rel_pose_Norm(1,1) < submap_params.camera_reading_angle && cam_rel_pose_Norm(1,2) < submap_params.camera_reading_distance)
        small_movement_id = [small_movement_id; i];
        %fprintf("small movement, camera: %d, pos: %03f, angle: %03f, \n", i,cam_rel_pose_Norm(1,1), cam_rel_pose_Norm(1,2));
        continue;        
    end    
    
    origin_id = i;
    
    % record relative poses and pose norm
    relative_poses_counter = relative_poses_counter + 1;
    %fprintf('relative_pose_counter: %d\n', relative_poses_counter);
    cam_relative_poses{relative_poses_counter} = cam_rel_pose;
    cam_relative_poses_Norm(relative_poses_counter,:) = cam_rel_pose_Norm;
    
    % travelled distance
    sum_dist = sum_dist + cam_relative_poses_Norm(relative_poses_counter,2);
    
    if sum_dist <= submap_params.submap_cover_distance  
        submap_idx(relative_poses_counter) = submap_counter;
        continue;
    else
        sum_dist = 0;
        submap_counter = submap_counter + 1;
        submap_idx(relative_poses_counter) = submap_counter;
    end        

end

% remove camera timestamps with small movement
camera_timestamps(small_movement_id,:) = [];
end


