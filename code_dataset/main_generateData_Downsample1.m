close all
clear all
clc

%%
% load params
loadParams

%% step1: sychronize timestamps, thus both camera have the same timestamps, each camera has nearest laser id.
% camera_laser_idx{camera_id} = corresponding_laser_id
[camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps(camera_timestamps, laser_timestamps);

figure,
plot(laser_timestamps(:,1), 'r.')
hold on
plot(camera_laser_idx, camera_timestamps{1}(:,1), 'b.')
plot(camera_laser_idx, camera_timestamps{2}(:,1), 'g.')
title('synchronized camera and lidar timestamps')


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
[camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeSmallCamPoses(camera_timestamps, camera_laser_idx, cam_global_poses, camera_reading_angle, camera_reading_distance);

% %% step3-3: remove laser scans with small relative movement
% 
% 
% figure,
% plot(laser_timestamps(:,1), 'r.')
% hold on
% plot(camera_laser_idx, camera_timestamps{1}(:,1), 'b.')
% plot(camera_laser_idx, camera_timestamps{2}(:,1), 'g.')
% title('filtered camera and laser timestamps (without small motion)')
% hold off


%% ########### step4: chunk id ############
%% step4-1: get submap indices (use only 1 camera as reference)
% camera_submap_idx(camera_id) = corresponding_submap_id
camera_submap_idx = get_camera_submap_indices(camera_timestamps{1}, cam_global_poses{1}, submap_cover_distance);

%% step4-2: get the laser scan idx for each submap
% submap_laser_idx{submapID} = [laser_id1, .... ]
submap_laser_idx = get_submap_laser_indices(camera_submap_idx, camera_laser_idx);

%% step4-3: for one submap example, visualize point cloud
% Build Pointcloud from random chunk
num_submap = size(submap_laser_idx,1);
%shuffle_id = randperm(num_submap);
%submapID = shuffle_id(1);
pcd_dir = strcat(base_path, '/', laser_submap_dir, '/', date_time);
if ~exist(pcd_dir, 'dir') 
    mkdir(pcd_dir);
end
%for submapID=1:num_submap
submapID = 1;
    pointcloud = build_pointcloud(G_ins_laser, laser_global_poses, ...
        submap_laser_idx, submapID, laser_timestamps, laser_dir);
%     pcd_path = sprintf('%s/%03d.pcd', pcd_dir, submapID); 
%     pcd_fig_path = sprintf('%s/%03d.png',pcd_dir, submapID);
%     writePCLtoPCD(pointcloud, pcd_path, pcd_fig_path);
%end

%% ########### step5: downsample pcl ##############
%%
%Remove ground plane
% [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.5);
% out_of_plane=pointcloud(:,out_plane);
off_groundplane_pointcloud = pointcloud;

%%
% downsample points with grid filter
pointcloud_downsampled_temp = pcdownsample(pointCloud(off_groundplane_pointcloud'),'gridAverage',pointcloud_downsample_gridstep);
pointcloud_downsampled = pointcloud_downsampled_temp.Location';

% visualize downsampled pointcloud
figure,
scatter3(-pointcloud_downsampled(2,1:end), ...
         -pointcloud_downsampled(1,1:end),...
         -pointcloud_downsampled(3,1:end), 1, 'b.');
title('pointcloud downsampled')


%% ######## step6: 2D-3D correspondences ########
%% step6-1: get images in current submap/chunk (40m), extract SIFT 
% get camera frames in the submap
cam_idx = find(camera_submap_idx == submapID);

% Compute sift features for each image in the submap
% images{1=left/2=right}{img_id} = image
% image_features{1=left/2=right}{img_id} = sift_keypoint
images=cell(2,1);
image_features=cell(2,1);
for i=1:2
    fprintf('extracting sift features of camera %d ...\n', i);
    [images{i}, image_features{i}] = ...
        computeImageFeatures(cam_idx, camera_dir{i}, camera_timestamps{i}, LUT{i});
end

%% step6-2: project all 3D point to each image in the chunk, find nearest sift points 
% project 3d points into images
% reproj_point_img_idx_list{1=left/2=right}{imgID} = pointcloud_idx
% reproj_point_img_uv_list{1=left/2=right}{imgID} = uv_coordinates_on_image
reproj_point_img_idx_list=cell(2,1);
reproj_point_img_uv_list=cell(2,1);
for i=1:2
    %i=2;
    [reproj_point_img_idx_list{i}, reproj_point_img_uv_list{i}] = ...
        proj3DPoints2Img(images{i}, cam_idx, camera_laser_idx, cam_global_poses{i}, ...
        laser_global_poses, G_camera_image{i}, G_camera_ins{i}, pointcloud_downsampled, camera_intrinsics{i},i);
end

% search for correspondences
% list of img uv correspondences to the points 
% --> point_to_uv_correspondences{1=left/2=right}{pointID} = [imageID, reprojPts, featurePts; ...]
% each 3D point get a set of image points from different images
point_to_uv_correspondences=cell(2,1);
for i=1:2
    point_to_uv_correspondences{i} = ...
        getCorrespondences(images{i}, pointcloud_downsampled, cam_idx, image_features{i}, ...
        reproj_point_img_uv_list{i}, reproj_point_img_idx_list{i}, corrThreshold);
end

%% step6-3: for each 3D point, get all the image ids and keypoint position
% get image tracks that correspond to the 3d points 
% track_list{1=left/2=right} = [3d_pt_id, img_id, reproj_uv, sift_uv; ...]
% point IDs of points that are tracked --> track_point_idx
track_point_idx=cell(2,1);
track_list=cell(2,1);
for i=1:2
    [track_point_idx{i}, track_list{i}] = get_tracks_list(point_to_uv_correspondences{i}, track_length);
end

% visualise tracks
imgID_start=25;
imgID_end=32;

for i=1:2
    img_idx = find(track_list{i}(:,2) >= imgID_start & track_list{i}(:,2) <= imgID_end);
    point_tracks = track_list{i}(img_idx, :);
    pointIdx = unique(point_tracks(:,1));
 
    figure,
    imshow(images{i}{imgID_start})
    hold on
    for j=1:length(pointIdx)

        pointID = pointIdx(j);  
        idx = find( point_tracks(:,1) == pointID );
        uv = point_tracks(idx, 5:6);

        plot(uv(:,1), uv(:,2), 'g-')
        plot(uv(:,1), uv(:,2), 'r.')
    end
    title(['visualize tracks: ' num2str(i)]);
    pause(0.2)
    hold off
end


%% ######## step7: filter 3D points with high curvature ##############
% Find the normals, curvature and normalize curvature
[normals, curvature, normalized_curvature] = findPointNormals(off_groundplane_pointcloud',[],[0,0,0],true);

% set indicator for points with high curvature
% point ids of points with high curvature --> pointcloud_high_curvature_idx
% pointcloud_curvature_indicator{pointID} = 0 (low curvature), 1 (high curvature)
[pointcloud_high_curvature_idx, pointcloud_curvature_indicator] = ...
    getHighCurvature(off_groundplane_pointcloud, normalized_curvature, curvatureThresh);

% get high curvature points from original pointcloud
high_curvature_points = off_groundplane_pointcloud(:, pointcloud_curvature_indicator == 1);


track_high_curvature_point_idx=cell(2,1);
track_high_curvature_list=cell(2,1);
for i=1:2
    [track_high_curvature_point_idx{i}, track_high_curvature_list{i}] = ... 
        getHighCurvatureTracks(high_curvature_points, pointcloud_downsampled, track_point_idx{i}, track_list{i});
end

%% error: pointcloud_downsampled size & normalized_curvature size (same
% with pointcloud) different !!!!!
figure,
scatter3(-pointcloud_downsampled(2,1:end), ...
         -pointcloud_downsampled(1,1:end),...
         -pointcloud_downsampled(3,1:end), 1, normalized_curvature(1:end), '.');
hold on
scatter3(-pointcloud_downsampled(2,pointcloud_high_curvature_idx), ...
         -pointcloud_downsampled(1,pointcloud_high_curvature_idx),...
         -pointcloud_downsampled(3,pointcloud_high_curvature_idx), 5, 'r.');
hold off
axis equal

%% error: pointcloud_downsampled size & track_high_curvature_point_idx size
% (same with pointcloud) different !!!!!
% visualize 3d keypoints
figure,
scatter3(-pointcloud_downsampled(2,1:end), ...
         -pointcloud_downsampled(1,1:end),...
         -pointcloud_downsampled(3,1:end), 1, 'b.');
hold on
scatter3(-pointcloud_downsampled(2,track_high_curvature_point_idx{1}), ...
         -pointcloud_downsampled(1,track_high_curvature_point_idx{1}),...
         -pointcloud_downsampled(3,track_high_curvature_point_idx{1}), 8, 'r*');
     
scatter3(-pointcloud_downsampled(2,track_high_curvature_point_idx{2}), ...
         -pointcloud_downsampled(1,track_high_curvature_point_idx{2}),...
         -pointcloud_downsampled(3,track_high_curvature_point_idx{2}), 8, 'g*');

     
%% save pointcloud
% pcl = pointCloud(single(pointcloud'));
% pcl_downsample = pointCloud(single(pointcloud_downsampled'));
% figure,
% subplot(1,2,1),pcshow(pcl), title('original point cloud')
% subplot(1,2,2),pcshow(pcl_downsample), title('downsampled point cloud')
% pcwrite(pcl,'pcl_orig.pcd','Encoding','ascii');
% pcwrite(pcl_downsample,'pcl_downsample.pcd','Encoding','ascii');
% 
% % pcl normalization, zero-mean
% pcl_tmp = pointcloud';   % Nx3
% pcl_normalized = (pcl_tmp - repmat(mean(pcl_tmp), size(pcl_tmp,1),1))./repmat(std(pcl_tmp), size(pcl_tmp,1),1);
% pcl_normalized = pointCloud(single(pcl_normalized));
% figure, pcshow(pcl_normalized), title('normalized pcl')
% pcwrite(pcl_normalized, 'pcl_normalized.pcd', 'Encoding', 'ascii');

function writePCLtoPCD(pointcloud, pcd_path, pcd_fig_path)
% this function write pointcloud to PCD file
% pointcloud: 3xN
pcl = pointCloud(single(pointcloud)');
figure, pcshow(pcl),
saveas(gcf,pcd_fig_path);
pcwrite(pcl, pcd_path, 'Encoding', 'ascii');
end






%%