clear all
clc
mat_files = dir('*.mat');

% test run: {'04_2014-06-26-09-53-12.mat'}
% idx = [6,11,17,19,26,30,32,33,39,40];
% for i=3:4
%     fprintf('****** progress: %d/%d, run: %s\n', i, length(idx), mat_files(idx(i)).name);
%     test_mat_file = mat_files(idx(i)).name;
%     GenerateTestPatch(test_mat_file);
% end

idx=13;     % [13,18,21,28, 31]
fprintf('****** progress:run: %s\n', mat_files(idx).name);
test_mat_file = mat_files(idx).name;
GenerateTestPatch(test_mat_file);
    
% test_mat_file = mat_files(4).name;
% GenerateTestPatch(test_mat_file);

function GenerateTestPatch(mat_file_name)
%load(mat_files(mat_id).name)
load(mat_file_name)

sift_patch_dir = strcat(base_path, '/sift_patch_test/', date_time, '/');
iss_volume_dir = strcat(base_path, '/iss_volume_test/', date_time, '/');

% more params
% store poses with respect to the first frame in each submap
cam_global_pose_dir = strcat(base_path, '/cam_global_poses/', date_time, '/');
gt_sift_iss_dir = strcat(base_path,'/2d_3d_corr_gt/', date_time, '/');
sift_uv_txt_root = strcat(base_path,'/2d_3d_index_value/', date_time, '/');
iss_uv_txt_root = sift_uv_txt_root;
if ~exist(cam_global_pose_dir,'dir')
    mkdir(cam_global_pose_dir);
end

if ~exist(gt_sift_iss_dir,'dir')
    mkdir(gt_sift_iss_dir);
end

if ~exist(sift_uv_txt_root,'dir')
    mkdir(sift_uv_txt_root);
end


[~,name,~] = fileparts(mat_file_name);
fid = fopen(sprintf('logfile_v3/%s.txt',name),'a');
fprintf(fid, '********** Load file: %s**********\n',mat_file_name);
pcd_files = dir([pcd_dir '*.pcd']);
iss_files = dir([keypoints_dir '*.pcd']);
submap_keypoints = cell(num_submap,1);      %% keypoints in each submap
submap_pointclouds = cell(num_submap,1);    %% pointclouds without ground in each submap
submap_keypoint_laser_idx = cell(num_submap,1);
submap_keypoint_idx_in_pcl = cell(num_submap,1);
for submapID = 1:num_submap
    fprintf(fid, '    progress: %d/%d\n', submapID, num_submap);
    % submap pointcloud
    pointcloud = pcread(strcat(pcd_dir, pcd_files(submapID).name));
    submap_pointclouds{submapID} = (pointcloud.Location)';  % 3xN

    % submap keypoints
    key_points = pcread(strcat(keypoints_dir, iss_files(submapID).name));
    submap_keypoints{submapID} = (key_points.Location)';

    % find index of each keypoint from full submap pointcloud
    [common_rows, ia, ib]=intersect(submap_pointclouds{submapID}',submap_keypoints{submapID}', 'rows');
    if(length(ia) ~= size(submap_keypoints{submapID},2))
        fprintf(fid, '  Error indexing!\n');
        break;
    end
    [~,idx] = sort(ib,'ascend');
    submap_keypoint_idx_in_pcl{submapID} = ia(idx);
    submap_keypoint_laser_idx{submapID} = submap_point_laser_idx{submapID}(ia(idx));
end 

%% ########### step6: for each submap, project ISS points to each image, 2D-3D correspondences ########
fprintf(fid, ' ---Project ISS points to each image\n');
camera_idx = cell(num_submap,1);
for i=1:num_submap
    camera_idx{i} = find(camera_submap_idx == i);
end

for submap_id=1:num_submap
    submap_sift_patch_dir = sprintf('%s%03d',sift_patch_dir, submap_id);
    submap_iss_volume_dir = sprintf('%s%03d',iss_volume_dir, submap_id);
    % image poses in each submap
    submap_cam_poses_dir = sprintf('%s%03d',cam_global_pose_dir, submap_id);
    submap_sift_iss_corr_dir = sprintf('%s%03d', gt_sift_iss_dir, submap_id);

    if ~exist(submap_sift_patch_dir, 'dir') 
        mkdir(submap_sift_patch_dir);
    end

    if ~exist(submap_iss_volume_dir, 'dir') 
        mkdir(submap_iss_volume_dir);
    end
    
    if ~exist(submap_cam_poses_dir, 'dir')
        mkdir(submap_cam_poses_dir);
    end
    
    if ~exist(submap_sift_iss_corr_dir, 'dir')
        mkdir(submap_sift_iss_corr_dir);
    end

end

%%
selected_id = [116,118,120];
% start_id = selected_id(1);
% end_id = selected_id(end);
%start_id = ceil(num_submap*0.9) +1;
%end_id = (start_id+6)
for ii=1:length(selected_id)
%for submapID=start_id:end_id
    submapID = selected_id(ii);
    fprintf(fid, '  process submap %d, progress %d/%d\n',submapID,submapID,num_submap);
    cam_idx = camera_idx{submapID};
    
    submap_cam_global_poses_dir = sprintf('%s%03d',cam_global_pose_dir, submapID);
    submap_sift_patch_dir = sprintf('%s%03d',sift_patch_dir, submapID);
    submap_iss_volume_dir = sprintf('%s%03d',iss_volume_dir, submapID);
    submap_sift_iss_gt_corr_dir = sprintf('%s%03d',gt_sift_iss_dir, submapID);
    
    % extract sift patches
    % requirement: sift points dist>28, sift scale<=2.5
    images=cell(2,1);
    image_features=cell(2,1);
    for i=1:2
%         [images{i}, image_features{i}] = ...
%             ExtractSIFTPatch(cam_idx, cam_global_poses{i}, camera_dir{i}, camera_timestamps{i}, LUT{i},...
%             submapID, submap_cam_global_poses_dir,submap_sift_patch_dir, i,laser_global_poses, camera_laser_idx);
        [images{i}, image_features{i}] = ...
            ExtractSIFTPatch_v2(cam_idx, cam_global_poses{i}, camera_dir{i}, camera_timestamps{i}, LUT{i},...
            submapID, submap_cam_global_poses_dir,submap_sift_patch_dir, i,laser_global_poses, camera_laser_idx,...
            sift_uv_txt_root);
    end
            
    % extract iss volumes
    % requirement: points in volume >= 100
%     valid_keypoints_index = ExtractISSVolume(submap_keypoints{submapID}, submap_pointclouds{submapID}, submap_iss_volume_dir, ...
%         volume_radius,submapID);
    valid_keypoints_index = ExtractISSVolume_v2(submap_keypoints{submapID}, submap_pointclouds{submapID}, ...
        submap_iss_volume_dir, volume_radius,submapID, iss_uv_txt_root);   

    % project 3d points into images
    % reproj_point_img_idx_list{1=left/2=right}{imgID} = pointcloud_idx
    % reproj_point_img_uv_list{1=left/2=right}{imgID} = uv_coordinates_on_image
    fprintf(fid, '    Project nearby pointcloud to each image.\n');
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
    % --> point_to_uv_correspondences{1=left/2=right}{pointID} = [imageID, reprojPts, featurePts, featureScale; ...]
    % each 3D point get a set of image points from different images
    % getCorrespondences_v2: returns sift_to_iss_corespondences{1=left/2=right}{imageID} = [sift_id, sift_uv, iss_id, iss_uv]
    fprintf(fid, '    Search for 2d-3d correspondences.\n');
    point_to_uv_correspondences=cell(2,1);
    for i=1:2
        point_to_uv_correspondences{i} = ...
            getCorrespondences_v2(images{i}, submap_keypoints{submapID}, cam_idx, image_features{i}, ...
            reproj_point_img_uv_list{i}, reproj_point_img_idx_list{i}, corrThreshold, ...
            valid_keypoints_index, submapID, i, submap_sift_iss_gt_corr_dir);
    end
    
end

fclose(fid);
end