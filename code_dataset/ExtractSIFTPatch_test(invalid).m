% Compute sift features for each image in the submap

function ExtractSIFTPatch_test(cam_idx, cam_global_poses, camera_dir, camera_timestamps, LUT,...
    submapID, cam_global_pose_dir,submap_sift_patch_dir, cam_id, ...
    laser_global_poses, camera_laser_idx)

% images=cell(length(cam_idx), 1);
% image_features=cell(length(cam_idx), 1);


% write image poses in current submap
% submap_image_poses = cam_global_poses(cam_idx);
% pose_path = sprintf('%s/cam%d_%03d.mat', cam_global_pose_dir, cam_id,submapID);
% save(pose_path, 'submap_image_poses');
first_camId = cam_idx(1);
% cam_relative_poses = zeros(length(cam_idx),6);
cam_relative_poses = zeros(length(cam_idx),12);

for imgID=1:length(cam_idx)
    % camera pose relative to the first frame in the submap
    cur_frame_pose = cam_global_poses{cam_idx(imgID)};
    cur_frame_rel_pose = laser_global_poses{camera_laser_idx(first_camId)}\cur_frame_pose;  %4x4
    cur_frame_rel_pose_tvec = cur_frame_rel_pose(1:3,4)';    % 1x3 translation vector
    cur_frame_rel_pose_rvec = reshape(cur_frame_rel_pose(1:3,1:3)', 1,[]);  % 1x9 rotation vector
    cam_relative_poses(imgID,:) = [cur_frame_rel_pose_tvec, cur_frame_rel_pose_rvec]; % [tvec, rvec]
end

% write pose to file
pose_file_path = sprintf('%s/cam%d_poses.txt', cam_global_pose_dir, cam_id); 
dlmwrite(pose_file_path, cam_relative_poses, 'precision', '%.6f');
