clear all
clc

% load params
base_path = '/media/mengdan/data3/robotcar/grasshopper';
sift_iss_correspondences = '2d_3d_correspondences';
date_time = '2014-06-26-09-53-12';

sift_patch_dir = strcat(base_path, '/sift_patch/', date_time, '/');
iss_volume_dir = strcat(base_path, '/iss_volume/', date_time, '/');
sift_iss_correspondences_dir = strcat(base_path, '/', sift_iss_correspondences, '/', date_time);

%camera
camera{1}='mono_left';
camera{2}='mono_right';
camera_dir{1}=strcat(base_path, '/', camera{1},'/', date_time, '/', camera{1});
camera_dir{2}=strcat(base_path, '/', camera{2},'/', date_time, '/', camera{2});

camera_models_path = strcat(base_path, '/', 'robotcar-dataset-sdk/models/');

%%%%%%%%%%%%%Camera Model%%%%%%%%%%%%%
% load camera intrinsics
% fx, fy: horizontal/vertical focal length 
% cx, cy: horizontal/vertical principal point
image_size = 1024;
LUT=cell(2,1);
G_camera_image=cell(2,1);
[ fx, fy, cx, cy, G_camera_image{1}, LUT{1}] = ReadCameraModel(camera_dir{1},camera_models_path);
intrinsic_matrix = [fx,0,cx;0,fy,cy;0,0,1];
camera_intrinsics{1}=cameraParameters('IntrinsicMatrix',intrinsic_matrix); 
[ fx, fy, cx, cy, G_camera_image{2}, LUT{2}] = ReadCameraModel(camera_dir{2},camera_models_path);
intrinsic_matrix = [fx,0,cx;0,fy,cy;0,0,1];
camera_intrinsics{2}=cameraParameters('IntrinsicMatrix',intrinsic_matrix); 

% estimate each camera pose within current submap
submap_id = 2;
submap_sift_patch_dir = sprintf('%s%03d',sift_patch_dir, submap_id);
submap_iss_volume_dir = sprintf('%s%03d',iss_volume_dir, submap_id);

% 
cam_pose_root = sprintf('%s/cam_global_poses/%s/%03d',base_path, date_time,submap_id);
sift_iss_corr_files = dir([sift_iss_correspondences_dir '/*.txt']);
loc_errors = zeros(length(sift_iss_corr_files),1);
for i=1:length(sift_iss_corr_files)
    fprintf('Progress: %d/%d\n', i, length(sift_iss_corr_files));
    C = strsplit(sift_iss_corr_files(i).name,'_');
    cam_id = str2num(C{2}(4));
    [~,name,~]=fileparts(C{3});
    submap_image_id = str2num(name);

    loc_errors(i)=estimateCameraPose(cam_pose_root,submap_id, cam_id, submap_image_id, ...
        submap_sift_patch_dir, submap_iss_volume_dir,sift_iss_correspondences_dir,...
        camera_intrinsics{cam_id});    
end

% % load ground truth poses
% % submap_image_poses
% cam_id = 1;     % 1:left, 2:right camera
% submap_image_id = 21;
% cam_global_poses_path = sprintf('%s/cam_global_poses/%s/%03d/cam%d_%03d.mat',base_path, date_time,submap_id,cam_id,submap_id);
% load(cam_global_poses_path)
% 
% cam_relative_poses = cell(length(submap_image_poses),1);
% initial_pose = submap_image_poses{1};
% for i=1:length(submap_image_poses)
%     cam_relative_poses{i} = initial_pose \ submap_image_poses{i};
% end
%     
% % load sift_uv 
% % sift_pos{sift_id} = [sift_id;sift_uv];
% sift_uv_path = sprintf('%s/cam%d_%03d/%03d_%03d.mat',submap_sift_patch_dir,...
%     cam_id,submap_image_id,submap_id, submap_image_id);
% load(sift_uv_path)
% 
% % load iss_xyz
% % iss_xyz{iss_id} = [iss_id, iss_xyz];
% iss_xyz_path = sprintf('%s/%03d.mat', submap_iss_volume_dir,submap_id);
% load(iss_xyz_path)
% 
% % load network predictions
% sift_iss_file = sprintf('%s/%03d_cam%d_%03d.txt', sift_iss_correspondences_dir, submap_id, cam_id, submap_image_id);
% fid = fopen(sift_iss_file, 'r');
% lines = textscan(fid,'%d %d %s %s'); 
% fclose(fid);
% 
% imagePoints=zeros(length(lines{1}),2);
% for i=1:length(lines{1})
%     imagePoints(i,:) = sift_pos{lines{1}(i)}(2:3);
% end
% 
% worldPoints=zeros(length(lines{2}),3);
% for i=1:length(lines{1})
%     worldPoints(i,:) = iss_xyz{lines{2}(i)}(2:4);
% end
% 
% % estimate camera pose
% [worldOrientation,worldLocation, inlierIdx,status] = estimateWorldCameraPose(imagePoints,worldPoints,camera_intrinsics{cam_id},...
%     'MaxNumTrials', 10000,'Confidence', 99, 'MaxReprojectionError', 100);
% disp(sum(inlierIdx))
% disp(status)
% disp(worldLocation)
% disp(cam_relative_poses{submap_image_id})