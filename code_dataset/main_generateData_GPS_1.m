close all
clear all
clc

%%
% load params
loadGPSParams

%% step1: sychronize timestamps, thus both camera have the same timestamps, each camera has nearest laser id.
% camera_laser_idx{camera_id} = corresponding_laser_id
[camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps(camera_timestamps, laser_timestamps);

% figure,
% plot(laser_timestamps(:,1), 'r.')
% hold on
% plot(camera_laser_idx, camera_timestamps{1}(:,1), 'b.')
% plot(camera_laser_idx, camera_timestamps{2}(:,1), 'g.')
% title('synchronized camera and lidar timestamps')


%% ########## step2: global poses #########
%% step2-1: get global poses for all camera 
% get all camera global poses
cam_global_poses = cell(2,1);
for i=1:2
    cam_global_poses{i} = cell(size(camera_timestamps{i},1),1);
    for chunk=1:camera_timestamps{i}(end,2)
        fprintf('Processing chunk: %d camera %d poses\n', chunk, i)
        idx1 = size(find(camera_timestamps{i}(:,2) < chunk),1);
        idx2 = size(find(camera_timestamps{i}(:,2) <= chunk),1);
        poses = getGlobalPoses(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps{i}( camera_timestamps{i}(:,2) == chunk, 1)');    
        cam_global_poses{i}(idx1+1:idx2) = poses;    
    end
end

%% step2-2: get global poses for all laser 
laser_global_poses = cell(size(laser_timestamps,1),1);
for chunk=1:laser_timestamps(end,2)
    fprintf('Processing chunk: %d laser poses\n', chunk)
    idx1 = size(find(laser_timestamps(:,2) < chunk),1);
    idx2 = size(find(laser_timestamps(:,2) <= chunk),1);
    poses=getGlobalPoses(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), laser_timestamps( laser_timestamps(:,2) == chunk, 1)'); 
    laser_global_poses(idx1+1:idx2) = poses;    
end


%% ############ step3: filter camera poses #########
%% step3-1: remove cameras without camera and/or laser INS readings 
 [camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeCamPoseWOINS(camera_timestamps, camera_laser_idx, cam_global_poses, laser_global_poses);

%% step3-2: remove camera poses with small relative movement
[camera_timestamps, ~, cam_global_poses] = ...
    removeSmallCamPoses(camera_timestamps, camera_laser_idx, cam_global_poses, camera_reading_angle, camera_reading_distance);

%% step3-3: remove laser scans with small relative movement
[laser_timestamps, camera_laser_idx, laser_global_poses] = ...
    removeSmallLaserPoses(laser_timestamps, camera_timestamps, ...
    laser_global_poses, laser_reading_angle, laser_reading_distance);


%% ########### step4: chunk id ############
% step4-1: get submap indices (use only 1 camera as reference)
% camera_submap_idx(camera_id) = corresponding_submap_id
camera_submap_idx = get_camera_submap_indices(camera_timestamps{1}, cam_global_poses{1}, submap_cover_distance);

% step4-2: get the laser scan idx for each submap
% submap_laser_idx{submapID} = [laser_id1, .... ]
submap_laser_idx = get_submap_laser_indices(camera_submap_idx, camera_laser_idx);

% step4-3: for each submap, build pcl, remove ground, write to PCD
num_submap = size(submap_laser_idx,1);
submap_point_laser_idx = cell(num_submap,1);
for submapID=1:num_submap
    % build pcl for current submap
    [pointcloud,point_laser_idx] = build_pointcloud_withID(G_ins_laser, laser_global_poses, ...
        submap_laser_idx, submapID, laser_timestamps, laser_dir);
    
    % remove ground 
    [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.6);
    out_of_plane= pointcloud(:,out_plane);
    out_of_plane_idx = point_laser_idx(out_plane);
    
    submap_point_laser_idx{submapID} = out_of_plane_idx;
    
    % save pcl as pcd
    pcd_path = sprintf('%s/%03d.pcd', pcd_dir, submapID); 
    pcd_fig_path = sprintf('%s/%03d.png',pcd_dir, submapID);
    writePCLtoPCD(out_of_plane, pcd_path, pcd_fig_path);
end

%% Check the submaps with jump!!!!

%% ****** STOP HERE for ISS keypoints detection, using c++
% ******* STOP, wait for keypoints *********
% ****** STOP HERE for ISS keypoints detection, using c++


%% ########### step5: load ISS keypoints for each submap ########
pcd_files = dir([pcd_dir '*.pcd']);
iss_files = dir([keypoints_dir '*.pcd']);
submap_keypoints = cell(num_submap,1);      %% keypoints in each submap
submap_pointclouds = cell(num_submap,1);    %% pointclouds without ground in each submap
submap_keypoint_laser_idx = cell(num_submap,1);
submap_keypoint_idx_in_pcl = cell(num_submap,1);
for submapID = 1:num_submap
    fprintf('    progress: %d/%d\n', submapID, num_submap);
    % submap pointcloud
    pointcloud = pcread(strcat(pcd_dir, pcd_files(submapID).name));
    submap_pointclouds{submapID} = (pointcloud.Location)';  % 3xN
    %green color
%     pointcloud_color = uint8(zeros(pointcloud.Count,3));
%     pointcloud_color(:,1) = 0;
%     pointcloud_color(:,2) = 255;
%     pointcloud_color(:,3) = 0;
%     pointcloud.Color = pointcloud_color;      
    
    % submap keypoints
    key_points = pcread(strcat(keypoints_dir, iss_files(submapID).name));
    submap_keypoints{submapID} = (key_points.Location)';
    % red color
%     key_points_color = uint8(zeros(key_points.Count,3));
%     key_points_color(:,1)=255;
%     key_points_color(:,2)=0;
%     key_points_color(:,3)=51;     
%     key_points.Color=key_points_color;
%     
%     figure(1), pcshow(pointcloud);
%     hold on
%     pcshow(key_points, 'MarkerSize',15);
%     hold off
    %saveas(gcf, sprintf('%s/%03d.png',submap_keypoints_dir,submapID));
%     pause(0.1);

    % find index of each keypoint from full submap pointcloud
    [common_rows, ia, ib]=intersect(submap_pointclouds{submapID}',submap_keypoints{submapID}', 'rows');
    [~,idx] = sort(ib,'ascend');
    submap_keypoint_idx_in_pcl{submapID} = ia(idx);
    submap_keypoint_laser_idx{submapID} = submap_point_laser_idx{submapID}(ia(idx));
end 

%% ########### step6: for each submap, project ISS points to each image2D-3D correspondences ########
%submap_info = cell(num_submap,1);
camera_idx = cell(num_submap,1);
for i=1:num_submap
    camera_idx{i} = find(camera_submap_idx == i);
end

for submap_id=1:num_submap
    submap_sift_patch_dir = sprintf('%s%03d',sift_patch_dir, submap_id);
    submap_iss_volume_dir = sprintf('%s%03d',iss_volume_dir, submap_id);
    submap_proj_dir = sprintf('%s%03d',sift_iss_proj_dir, submap_id);
    
    if ~exist(submap_sift_patch_dir, 'dir') 
        mkdir(submap_sift_patch_dir);
    end
    
    if ~exist(submap_iss_volume_dir, 'dir') 
        mkdir(submap_iss_volume_dir);
    end
    
    if ~exist(submap_proj_dir, 'dir') 
        mkdir(submap_proj_dir);
    end
end

for submapID=1:num_submap
    fprintf('  process each submap, progress %d/%d\n',submapID,num_submap);
    cam_idx = camera_idx{submapID};
    
    % Compute sift features for each image in the submap
    % images{1=left/2=right}{img_id} = image
    % image_features{1=left/2=right}{img_id} = sift_keypoint
    images=cell(2,1);
    image_features=cell(2,1);
    fprintf('    Compute sift features.\n');
    for i=1:2        
        [images{i}, image_features{i}] = ...
            computeImageFeatures(cam_idx, camera_dir{i}, camera_timestamps{i}, LUT{i});
    end
    
    % project 3d points into images
    % reproj_point_img_idx_list{1=left/2=right}{imgID} = pointcloud_idx
    % reproj_point_img_uv_list{1=left/2=right}{imgID} = uv_coordinates_on_image
    fprintf('    Project nearby pointcloud to each image.\n');
    reproj_point_img_idx_list=cell(2,1);
    reproj_point_img_uv_list=cell(2,1);
    for i=1:2   
        [reproj_point_img_idx_list{i}, reproj_point_img_uv_list{i}] = ...
            projNeighbor3DPoints2Img(images{i}, cam_idx, camera_laser_idx, cam_global_poses{i}, ...
            laser_global_poses, G_camera_image{i}, G_camera_ins{i}, submap_keypoints{submapID}, ...
            camera_intrinsics{i},i,submap_keypoint_laser_idx{submapID},submapID,pcl_proj_dir); 
    end

    % search for correspondences
    % list of img uv correspondences to the points 
    % --> point_to_uv_correspondences{1=left/2=right}{pointID} = [imageID, reprojPts, featurePts; ...]
    % each 3D point get a set of image points from different images
    fprintf('    Search for 2d-3d correspondences.\n')
    point_to_uv_correspondences=cell(2,1);
    for i=1:2
        point_to_uv_correspondences{i} = ...
            getCorrespondences(images{i}, submap_keypoints{submapID}, cam_idx, image_features{i}, ...
            reproj_point_img_uv_list{i}, reproj_point_img_idx_list{i}, corrThreshold);
    end
    
    % get image tracks that correspond to the 3d points 
    % track_list{1=left/2=right} = [3d_pt_id, img_id, reproj_uv, sift_uv; ...]
    % point IDs of points that are tracked --> track_point_idx
    fprintf('    Get keypoint tracks on images. \n')
    track_point_idx=cell(2,1);
    track_list=cell(2,1);
    for i=1:2
        [track_point_idx{i}, track_list{i}] = get_tracks_list(point_to_uv_correspondences{i}, track_length);
    end
    
    % extract sift patch and iss volume
    fprintf('    Extract sift patch and iss volume.\n')
    submap_sift_patch_dir = sprintf('%s%03d',sift_patch_dir, submapID);
    submap_iss_volume_dir = sprintf('%s%03d',iss_volume_dir, submapID);
    submap_proj_dir = sprintf('%s%03d',sift_iss_proj_dir, submapID);
    for i=1:2  
        Extract_SIFT_ISS_Area(track_list{i}, track_point_idx{i}, images{i}, i, ...
            submap_keypoint_idx_in_pcl{submapID}, submap_pointclouds{submapID},volume_radius,...
            cam_global_poses{i}, laser_global_poses, camera_laser_idx, cam_idx,G_camera_image{i}, G_camera_ins{i}, camera_intrinsics{i},...
            submap_sift_patch_dir, submap_iss_volume_dir,submap_proj_dir);
    end    
    
    fprintf('*********** Submap %02d done.************ \n', submapID);    
end

%% ###### step7: for each submap, extract image patch (SIFT keypoints) and pointcloud volumn (ISS keypoints) ####
