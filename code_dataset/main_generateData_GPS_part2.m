%% ########### step5: load ISS keypoints for each submap ########
%load('05_2014-07-14-14-49-50.mat'); 

clear all
clc

mat_files = dir('*.mat');
for mat_id=4:40
    load(mat_files(mat_id).name)
    fprintf('---Load submap %d ISS keypoints, %s\n',mat_id, mat_files(mat_id).name);
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

        % submap keypoints
        key_points = pcread(strcat(keypoints_dir, iss_files(submapID).name));
        submap_keypoints{submapID} = (key_points.Location)';

        % find index of each keypoint from full submap pointcloud
        [common_rows, ia, ib]=intersect(submap_pointclouds{submapID}',submap_keypoints{submapID}', 'rows');
        if(length(ia) ~= size(submap_keypoints{submapID},2))
            fprintf('  Error indexing!\n');
            break;
        end
        [~,idx] = sort(ib,'ascend');
        submap_keypoint_idx_in_pcl{submapID} = ia(idx);
        submap_keypoint_laser_idx{submapID} = submap_point_laser_idx{submapID}(ia(idx));
    end 

    %% ########### step6: for each submap, project ISS points to each image, 2D-3D correspondences ########
    fprintf('---Project ISS points to each image\n');
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

    %%
    for submapID=1:num_submap-1
        fprintf('  process submap %d, progress %d/%d\n',submapID,submapID,num_submap);
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
        % --> point_to_uv_correspondences{1=left/2=right}{pointID} = [imageID, reprojPts, featurePts, featureScale; ...]
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
            [track_point_idx{i}, track_list{i}] = removeCloseSIFTPoints(track_list{i}, length(images{i}));
        end
        

        % extract sift patch and iss volume
        fprintf('    Extract sift patch and iss volume.\n')
        submap_sift_patch_dir = sprintf('%s%03d',sift_patch_dir, submapID);
        submap_iss_volume_dir = sprintf('%s%03d',iss_volume_dir, submapID);
        submap_proj_dir = sprintf('%s%03d',sift_iss_proj_dir, submapID);
        for i=1:2  
            Extract_SIFT_ISS_Area_With_Scale(track_list{i}, track_point_idx{i}, images{i}, i, ...
            submap_keypoint_idx_in_pcl{submapID}, submap_pointclouds{submapID},volume_radius,...
            cam_global_poses{i}, laser_global_poses, camera_laser_idx, cam_idx,G_camera_image{i}, G_camera_ins{i}, camera_intrinsics{i},...
            submap_sift_patch_dir, submap_iss_volume_dir,submap_proj_dir);
        end      
    end
    
    close all
    clearvars submap_keypoints submap_pointclouds images image_features 
end