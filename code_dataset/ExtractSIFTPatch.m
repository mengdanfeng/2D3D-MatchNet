% Compute sift features for each image in the submap

function [images, image_features, patch_paths]=ExtractSIFTPatch(cam_idx, cam_global_poses, camera_dir, camera_timestamps, LUT,...
    submapID, cam_global_pose_dir,submap_sift_patch_dir, cam_id, ...
    laser_global_poses, camera_laser_idx)

% images=cell(length(cam_idx), 1);
% image_features=cell(length(cam_idx), 1);

patch_size = 256;
half_size = patch_size/2;

% write image poses in current submap
% submap_image_poses = cam_global_poses(cam_idx);
% pose_path = sprintf('%s/cam%d_%03d.mat', cam_global_pose_dir, cam_id,submapID);
% save(pose_path, 'submap_image_poses');
first_camId = cam_idx(1);
cam_relative_poses = zeros(length(cam_idx),12);

images=cell(length(cam_idx), 1);
image_features=cell(length(cam_idx), 1);

for imgID=1:length(cam_idx)

    % camera pose relative to the first frame in the submap
    cur_frame_pose = cam_global_poses{cam_idx(imgID)};
    cur_frame_rel_pose = laser_global_poses{camera_laser_idx(first_camId)}\cur_frame_pose;  %4x4
    cur_frame_rel_pose_tvec = cur_frame_rel_pose(1:3,4)';    % 1x3 translation vector
    cur_frame_rel_pose_rvec = reshape(cur_frame_rel_pose(1:3,1:3)', 1,[]);  % 1x9 rotation vector
    cam_relative_poses(imgID,:) = [cur_frame_rel_pose_tvec, cur_frame_rel_pose_rvec]; % [tvec, rvec]
    
    % compute sift features
    fprintf('      compute sift: %d of %d images\n', imgID, length(cam_idx));
    I = LoadImage(camera_dir, camera_timestamps(cam_idx(imgID),1), LUT);
    if isempty(I)
        fprintf('  image does not exist. dir:%s, time: %f\n', camera_dir,camera_timestamps(cam_idx(imgID)));
    end
    image_size = size(I);
    I_gray= single(rgb2gray(I));
    f=vl_sift(I_gray);
    
%     figure,
%     imshow(I)
%     hold on
%     plot(f(1,:), f(2,:), 'g.')
%     hold off
%     pause

    % remove close SIFT points
%    sift_points = f(1:2,:)';  % Nx2   
%     delete_list = zeros(size(sift_points,1),1);
%     dist = pdist2(sift_points, sift_points);    % NxN
%     for i=1:size(sift_points,1)
%         if delete_list(i) ==0
%             id = find(dist(i,(i+1):end) <= 32);
%             delete_list(id+i) = 1;
%         end
%     end
%     
%     if sum(delete_list==0) == size(sift_points,1)
%         continue;
%     end
%     
%     f = f(:,delete_list==0);
    
    
    % remove sift features with large scale
    sift_scales = f(3,:)/1.6;
%     if(min(sift_scales)<1)
%         fprintf(' Error: sift scale smaller than 1!\n');
%         break;
%     end
    f = f(:,sift_scales<=2.5);    

    % extract sift patches
    sift_patches = {};
    patch_paths = {};
    sift_pos = {};
    sift_id = 0;
    valid_index = [];
    for i=1:size(f,2)
        sift_uv = f(1:2,i);
        sift_scale = f(3,i);
        scaled_half_size = half_size/sift_scale;

        top_left_x = round(sift_uv(1) - scaled_half_size);
        top_left_y = round(sift_uv(2) - scaled_half_size);
        bottom_right_x = round(sift_uv(1) + scaled_half_size);
        bottom_right_y = round(sift_uv(2) + scaled_half_size);
        if(top_left_x < 1 || top_left_y < 1 || bottom_right_x > image_size(1) || bottom_right_y > image_size(2))
            %fprintf(' out of range.\n');
            continue;
        end        
        
        % remove patch with small variance    
        sift_patch = I(top_left_y:bottom_right_y,top_left_x:bottom_right_x, :);
        gray_patch = double(rgb2gray(sift_patch));
        patch_var = var(gray_patch(:));
        if patch_var<=300
            continue;
        end
        
        valid_index = [valid_index;i];
                
        sift_id = sift_id + 1;
        sift_patches{sift_id} = sift_patch;

        patch_dir = sprintf('%s/cam%d_%03d', submap_sift_patch_dir,cam_id,imgID);
        if ~exist(patch_dir, 'dir')
            mkdir(patch_dir);
        end
        patch_path = sprintf('%s/%03d_%03d_%05d.png', patch_dir,submapID, imgID,  sift_id);
        patch_paths{sift_id} = patch_path;
        sift_pos{sift_id} = {sift_id;patch_path;sift_uv};
    end


    % return images and sift_features
    images{imgID}=I;
    image_features{imgID}=f(:, valid_index);
    
    % write sift patches
    cellfun(@imwrite, sift_patches, patch_paths); 
    sift_uv_path = sprintf('%s/cam%d_%03d/%03d_%03d.mat',submap_sift_patch_dir,...
        cam_id,imgID,submapID, imgID);
    save(sift_uv_path, 'sift_pos');

    
end      

% write pose to file
pose_file_path = sprintf('%s/cam%d_poses.txt', cam_global_pose_dir, cam_id); 
dlmwrite(pose_file_path, cam_relative_poses, 'precision', '%.6f');
