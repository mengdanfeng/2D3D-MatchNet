clear all
clc
mat_files = dir('*.mat');

% test run: {'04_2014-06-26-09-53-12.mat'}
test_mat_file = mat_files(4).name;
GenerateTestPatch(test_mat_file);

function GenerateTestPatch(mat_file_name)
%load(mat_files(mat_id).name)
load(mat_file_name)

% more params
% store poses with respect to the first frame in each submap
cam_global_pose_dir = strcat(base_path, '/cam_global_poses/', date_time, '/');
gt_sift_iss_dir = strcat(base_path,'/2d_3d_corr_gt/', date_time, '/');
if ~exist(cam_global_pose_dir,'dir')
    mkdir(cam_global_pose_dir);
end

if ~exist(gt_sift_iss_dir,'dir')
    mkdir(gt_sift_iss_dir);
end

[~,name,~] = fileparts(mat_file_name);
fid = fopen(sprintf('logfile_v2/%s.txt',name),'a');
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
for submapID=1:num_submap-1
    %fprintf(fid, '  process submap %d, progress %d/%d\n',submapID,submapID,num_submap);
    disp(submapID)
    cam_idx = camera_idx{submapID};
    
    submap_cam_global_poses_dir = sprintf('%s%03d',cam_global_pose_dir, submapID);
    submap_sift_patch_dir = sprintf('%s%03d',sift_patch_dir, submapID);
    submap_iss_volume_dir = sprintf('%s%03d',iss_volume_dir, submapID);
    submap_sift_iss_gt_corr_dir = sprintf('%s%03d',gt_sift_iss_dir, submapID);
    
    % extract sift patches
    % requirement: sift points dist>28, sift scale<=2.5

    for i=1:2
        ExtractSIFTPatch_test(cam_idx, cam_global_poses{i}, camera_dir{i}, camera_timestamps{i}, LUT{i},...
            submapID, submap_cam_global_poses_dir,submap_sift_patch_dir, i,laser_global_poses, camera_laser_idx);
    end
    
end

fclose(fid);
end