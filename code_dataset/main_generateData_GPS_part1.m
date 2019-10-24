close all
clear all
clc

%%
% list of different date and time
folders = dir('/media/mengdan/data3/robotcar/grasshopper/mono_left/');
folders = folders(3:end);

% load params
% jumped folders: 1,2,
%for folder_id = 1:length(folders)
for folder_id = 3:4
    
    folder_id = 4;
    
    date_time = folders(folder_id).name;
    fprintf('******** Now Processing folder %d: %s**********\n', folder_id, date_time);

    loadGPSParams_1

    %% step1: sychronize timestamps, thus both camera have the same timestamps, each camera has nearest laser id.
    % camera_laser_idx{camera_id} = corresponding_laser_id
    for i=1:2
        [camera_timestamps{i}] = removePoseWOData(camera_timestamps{i}, camera_dir{i}, 1);
        [camera_timestamps{i}] = synchronizeWithINS(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps{i});        
    end
    laser_timestamps = removePoseWOData(laser_timestamps, laser_dir, 2);
    laser_timestamps = synchronizeWithINS(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), laser_timestamps);
    
    [camera_timestamps, laser_timestamps, camera_laser_idx] = ...
        synchronizeTimeStamps1(camera_timestamps, laser_timestamps);


    %% ########## step2: global poses #########
    %% step2-1: get global poses for all camera 
    % get all camera global poses
    fprintf('---Get camera global poses\n');
    cam_global_poses = cell(2,1);
    for i=1:2
        cam_global_poses{i} = cell(size(camera_timestamps{i},1),1);
        for chunk=1:camera_timestamps{i}(end,2)
            fprintf('  Processing chunk: %d camera %d poses\n', chunk, i)
            idx1 = size(find(camera_timestamps{i}(:,2) < chunk),1);
            idx2 = size(find(camera_timestamps{i}(:,2) <= chunk),1);
            poses = getGlobalPoses(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), camera_timestamps{i}( camera_timestamps{i}(:,2) == chunk, 1)');    
            cam_global_poses{i}(idx1+1:idx2) = poses;    
        end
    end

    %% step2-2: get global poses for all laser 
    fprintf('---Get laser global poses\n');
    laser_global_poses = cell(size(laser_timestamps,1),1);
    for chunk=1:laser_timestamps(end,2)
        fprintf('  Processing chunk: %d laser poses\n', chunk)
        idx1 = size(find(laser_timestamps(:,2) < chunk),1);
        idx2 = size(find(laser_timestamps(:,2) <= chunk),1);
        poses=getGlobalPoses(strcat(base_path, '/gps/', date_time, '/gps/ins.csv'), laser_timestamps( laser_timestamps(:,2) == chunk, 1)'); 
        laser_global_poses(idx1+1:idx2) = poses;    
    end


    %% ############ step3: filter camera poses #########
    fprintf('---Filter camera and laser poses, remove small motion\n');
    %% step3-1: remove cameras without camera and/or laser INS readings 
     [camera_timestamps, camera_laser_idx, cam_global_poses] = ...
        removeCamPoseWOINS(camera_timestamps, camera_laser_idx, cam_global_poses, laser_global_poses);

    %% step3-2: remove camera poses with small relative movement
    [camera_timestamps, camera_laser_idx, cam_global_poses] = ...
        removeSmallCamPoses(camera_timestamps, camera_laser_idx, cam_global_poses, camera_reading_angle, camera_reading_distance);

    %% step3-3: remove laser scans with small relative movement
%     [laser_timestamps, camera_timestamps, camera_laser_idx, cam_global_poses, laser_global_poses] = ...
%         removeSmallLaserPoses(laser_timestamps, camera_timestamps, ...
%         laser_global_poses, laser_reading_angle, laser_reading_distance, ...
%         camera_laser_idx,cam_global_poses);


    %% ########### step4: chunk id ############
    % step4-1: get submap indices (use only 1 camera as reference)
    % camera_submap_idx(camera_id) = corresponding_submap_id
    fprintf('---Get all submaps for camera and laser\n');
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

    filename = sprintf('%02d_%s.mat',folder_id, date_time);
    save(filename);
    fprintf('\n\n');
    
    close all
end

%% Check the submaps with jump!!!!

%% ****** STOP HERE for ISS keypoints detection, using c++
% ******* STOP, wait for keypoints *********
% ****** STOP HERE for ISS keypoints detection, using c++
% goto /media/mengdan/data3/robotcar/grasshopper/code/ISS_Detector/build
% run  ./iss_detector date_time(2014-06-26-09-53-12) 0.1