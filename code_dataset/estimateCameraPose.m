function [loc_error] = estimateCameraPose(cam_pose_root,submap_id, cam_id, submap_image_id, ...
    submap_sift_patch_dir, submap_iss_volume_dir,sift_iss_correspondences_dir,...
    camera_intrinsics)
% load ground truth poses
% submap_image_poses
% cam_id = 1;     % 1:left, 2:right camera
% submap_image_id = 21;
cam_global_poses_path = sprintf('%s/cam%d_%03d.mat',cam_pose_root,cam_id,submap_id);
load(cam_global_poses_path)

cam_relative_poses = cell(length(submap_image_poses),1);
initial_pose = submap_image_poses{1};
for i=1:length(submap_image_poses)
    cam_relative_poses{i} = initial_pose \ submap_image_poses{i};
end
    
% load sift_uv 
% sift_pos{sift_id} = [sift_id;sift_uv];
sift_uv_path = sprintf('%s/cam%d_%03d/%03d_%03d.mat',submap_sift_patch_dir,...
    cam_id,submap_image_id,submap_id, submap_image_id);
load(sift_uv_path)

% load iss_xyz
% iss_xyz{iss_id} = [iss_id, iss_xyz];
iss_xyz_path = sprintf('%s/%03d.mat', submap_iss_volume_dir,submap_id);
load(iss_xyz_path)

% load network predictions
sift_iss_file = sprintf('%s/%03d_cam%d_%03d.txt', sift_iss_correspondences_dir, submap_id, cam_id, submap_image_id);
fid = fopen(sift_iss_file, 'r');
lines = textscan(fid,'%d %d %s %s'); 
fclose(fid);

if isempty(lines) || length(lines{1}) <= 6
    fprintf(" Empty image points!\n");
    loc_error = 1e10;
    return;
end

imagePoints=zeros(length(lines{1}),2);
for i=1:length(lines{1})
    imagePoints(i,:) = sift_pos{lines{1}(i)}(2:3);
end

worldPoints=zeros(length(lines{2}),3);
for i=1:length(lines{1})
    worldPoints(i,:) = iss_xyz{lines{2}(i)}(2:4);
end

% estimate camera pose
[worldOrientation,worldLocation, inlierIdx,status] = estimateWorldCameraPose(imagePoints,worldPoints,camera_intrinsics,...
    'MaxNumTrials', 10000,'Confidence', 99, 'MaxReprojectionError', 100);

% ground truth pose
gt_pose = cam_relative_poses{submap_image_id};
gt_location = gt(1:3, 4);

loc_error = sqrt(sum((gt_location - worldLocation).^2));

