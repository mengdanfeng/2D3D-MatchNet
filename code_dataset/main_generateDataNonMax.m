close all
clear all
clc

%%
% load params
loadParams

%%
% sychronize timestamps
% camera_laser_idx{camera_id} = corresponding_laser_id
[camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps(camera_timestamps, laser_timestamps);

figure,
plot(laser_timestamps(:,1), 'r.')
hold on
plot(camera_laser_idx, camera_timestamps{1}(:,1), 'b.')
plot(camera_laser_idx, camera_timestamps{2}(:,1), 'g.')

%% 
% get all camera global poses
cam_global_poses = cell(2,1);
for i=1:2
    cam_global_poses{i} = cell(size(camera_timestamps{i},1),1);
    for chunk=1:camera_timestamps{i}(end,2)
        fprintf('Processing chunk: %d camera %d poses\n', chunk, i)
        idx1 = size(find(camera_timestamps{i}(:,2) < chunk),1);
        idx2 = size(find(camera_timestamps{i}(:,2) <= chunk),1);
        poses = getGlobalPoses(strcat(camera_base_path,'/gps/ins.csv'), camera_timestamps{i}( camera_timestamps{i}(:,2) == chunk, 1)');    
        cam_global_poses{i}(idx1+1:idx2) = poses;    
    end
end

%%
% get all laser global poses
laser_global_poses = cell(size(laser_timestamps,1),1);
for chunk=1:laser_timestamps(end,2)
    fprintf('Processing chunk: %d laser poses\n', chunk)
    idx1 = size(find(laser_timestamps(:,2) < chunk),1);
    idx2 = size(find(laser_timestamps(:,2) <= chunk),1);
    poses=getGlobalPoses(strcat(base_path,'/gps/ins.csv'), laser_timestamps( laser_timestamps(:,2) == chunk, 1)'); 
    laser_global_poses(idx1+1:idx2) = poses;    
end

%%
% remove cameras without camera and/or laser INS readings 
 [camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeCamPoseWOINS(camera_timestamps, camera_laser_idx, cam_global_poses, laser_global_poses);

%%
% remove camera poses with small relative movement
[camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeSmallCamPoses(camera_timestamps, camera_laser_idx, cam_global_poses, camera_reading_angle, camera_reading_distance);

figure,
plot(laser_timestamps(:,1), 'r.')
hold on
plot(camera_laser_idx, camera_timestamps{1}(:,1), 'b.')
plot(camera_laser_idx, camera_timestamps{2}(:,1), 'g.')
hold off

%%
% get submap indices (use only 1 camera as reference)
% camera_submap_idx(camera_id) = corresponding_submap_id
camera_submap_idx = get_camera_submap_indices(camera_timestamps{1}, cam_global_poses{1}, submap_cover_distance);

%% 
% get the laser scan idx for each submap
% submap_laser_idx{submapID} = [laser_id1, .... ]
submap_laser_idx = get_submap_laser_indices(camera_submap_idx, camera_laser_idx);

%%
% Build Pointcloud
submapID=125;
pointcloud = build_pointcloud(G_ins_laser, laser_global_poses, ...
    submap_laser_idx, submapID, laser_timestamps, laser_dir);

%%
%Remove ground plane
% [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.5);
% out_of_plane=pointcloud(:,out_plane);
off_groundplane_pointcloud = pointcloud;


%%
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
        computeImageFeatures(cam_idx, camera_base_path, camera_dir{i}, camera_timestamps{i}, LUT{i});
end

%%
% project 3d points into images
% reproj_point_img_idx_list{1=left/2=right}{imgID} = pointcloud_idx
% reproj_point_img_uv_list{1=left/2=right}{imgID} = uv_coordinates_on_image
reproj_point_img_idx_list=cell(2,1);
reproj_point_img_uv_list=cell(2,1);
for i=1:2
    [reproj_point_img_idx_list{i}, reproj_point_img_uv_list{i}] = ...
        proj3DPoints2Img(images{i}, cam_idx, camera_laser_idx, cam_global_poses{i}, ...
        laser_global_poses, G_camera_image{i}, G_camera_ins{i}, off_groundplane_pointcloud, camera_intrinsics{i});
end

% search for correspondences
% list of img uv correspondences to the points 
% --> point_to_uv_correspondences{1=left/2=right}{pointID} = [imageID uv; ...]
point_to_uv_correspondences=cell(2,1);
for i=1:2
    point_to_uv_correspondences{i} = ...
        getCorrespondences(images{i}, off_groundplane_pointcloud, cam_idx, image_features{i}, ...
        reproj_point_img_uv_list{i}, reproj_point_img_idx_list{i}, corrThreshold);
end

%%
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
    hold off
end

%%
% Find the normals, curvature and normalize curvature
[normals, curvature, normalized_curvature] = findPointNormals(off_groundplane_pointcloud',[],[0,0,0],true);

% set indicator for points with high curvature
% point ids of points with high curvature --> pointcloud_high_curvature_idx
% pointcloud_curvature_indicator{pointID} = 0 (low curvature), 1 (high curvature)
[pointcloud_high_curvature_idx, pointcloud_curvature_indicator] = ...
    getHighCurvature(off_groundplane_pointcloud, normalized_curvature, curvatureThresh);

figure,
scatter3(-off_groundplane_pointcloud(2,1:end), ...
         -off_groundplane_pointcloud(1,1:end),...
         -off_groundplane_pointcloud(3,1:end), 10, normalized_curvature(1:end), '.');
axis equal

% get tracked points with non-maximally suppressed high curvature

% good_track_point_idx=cell(2,1); white or off-white background
% Taken in full-face view directly facing the camera
% With a neutral facial expression and both eyes open

good_track_point_idx=cell(2,1);
good_track_list=cell(2,1);

for i=1:2
   [good_track_point_idx{i}, good_track_list{i}] = ...
       nonMaximalSuppression(off_groundplane_pointcloud, normalized_curvature, track_point_idx{i}, track_list{i}, curvatureThresh);
end

figure,
scatter3(-pointcloud(2,1:end), ...
         -pointcloud(1,1:end),...
         -pointcloud(3,1:end), 5, 'g.');
hold on

scatter3(-pointcloud(2,good_track_point_idx{1}), ...
         -pointcloud(1,good_track_point_idx{1}),...
         -pointcloud(3,good_track_point_idx{1}), 15, 'r*');
     
 scatter3(-pointcloud(2,good_track_point_idx{2}), ...
          -pointcloud(1,good_track_point_idx{2}),...
          -pointcloud(3,good_track_point_idx{2}), 15, 'b*');
     
axis equal

