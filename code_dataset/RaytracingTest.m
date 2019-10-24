%% Project all the points (w/o ground) in one submap to each image
% project 3d points into images
% reproj_point_img_idx_list{1=left/2=right}{imgID} = pointcloud_idx
% reproj_point_img_uv_list{1=left/2=right}{imgID} = uv_coordinates_on_image
reproj_point_img_idx_list=cell(2,1);
reproj_point_img_uv_list=cell(2,1);
for i=1:2
    %i=2;
    [reproj_point_img_idx_list{i}, reproj_point_img_uv_list{i}] = ...
        proj3DPoints2Img(images{i}, cam_idx, camera_laser_idx, cam_global_poses{i}, ...
        laser_global_poses, G_camera_image{i}, G_camera_ins{i}, submap_pointclouds{submapID}, camera_intrinsics{i},i);
end

%% Project keypoints in one submap to each image
% project 3d points into images
% reproj_point_img_idx_list{1=left/2=right}{imgID} = pointcloud_idx
% reproj_point_img_uv_list{1=left/2=right}{imgID} = uv_coordinates_on_image
keypts_reproj_point_img_idx_list=cell(2,1);
keypts_reproj_point_img_uv_list=cell(2,1);
for i=1:2
    %i=2;
    [keypts_reproj_point_img_idx_list{i}, keypts_reproj_point_img_uv_list{i}] = ...
        proj3DPoints2Img(images{i}, cam_idx, camera_laser_idx, cam_global_poses{i}, ...
        laser_global_poses, G_camera_image{i}, G_camera_ins{i}, submap_keypoints{submapID}, camera_intrinsics{i},i);
end

%% convert each submap pointcloud to 
grid_resolution_x = 0.2;
grid_resolution_y = 0.2;
grid_resolution_z = 0.2;

%gridSize = [8,8,8];
for i=1:2
    %camera_center{i} =          % camera center's position in world coordinates
    for j=1:length(cam_idx) 
        camera_center_pose = cam_global_poses{i}{cam_idx(j)} \ laser_global_poses{camera_laser_idx(cam_idx(1))} * G_camera_ins{i};
        camera_center_position = camera_center_pose(1:3,4);
        
        pointcloud_idx = reproj_point_img_idx_list{i}{j};
        submap_in_img_points = submap_pointclouds{submapID}(:,pointcloud_idx);      % 3xN
        
        % visualize
        figure,pcshow(pointCloud(submap_in_img_points'));
        
        % pcdenoise (optional)
        submap_in_img_points_denoised = pcdenoise(pointCloud(submap_in_img_points'));
        submap_in_img_points_denoised = (submap_in_img_points_denoised.Location)';     % Nx3
        figure,pcshow(pointCloud(submap_in_img_points_denoised));
        
        % grid params
        min_x = min(submap_in_img_points_denoised(1,:));
        min_y = min(submap_in_img_points_denoised(2,:));
        min_z = min(submap_in_img_points_denoised(3,:));
        max_x = max(submap_in_img_points_denoised(1,:));
        max_y = max(submap_in_img_points_denoised(2,:));
        max_z = max(submap_in_img_points_denoised(3,:));
        
        %[min_x, min_y, min_z] = min(submap_in_img_points,[],2);
        grid_bounds = [min_x,min_y,min_z,max_x,max_y,max_z];      
        grid_size_x = (max_x - min_x)/grid_resolution_x;
        grid_size_y = (max_y - min_y)/grid_resolution_y;
        grid_size_z = (max_z - min_z)/grid_resolution_z;
        grid_size = [grid_size_x, grid_size_y, grid_size_z];
        
        % plot cube
        % plotCube()
        
        % line segment, start
        keypoints_idx = keypts_reproj_point_img_idx_list{i}{j};
        keypoints = submap_keypoints{submapID}(:,keypoints_idx);
        is_keypoint_blocked = zeros(size(keypoints,2),1);
        
        % keypoints grid position
        %bounding_box = zeros(grid_size_x, grid_size_y, grid_size_z);
        occupied_grid_x = floor((keypoints(1,:)-min_x)./grid_resolution_x) + 1;
        occupied_grid_y = floor((keypoints(2,:)-min_y)./grid_resolution_y) + 1;
        occupied_grid_z = floor((keypoints(3,:)-min_z)./grid_resolution_z) + 1;
        occupied_grid = [occupied_grid_x, occupied_grid_y, occupied_grid_z];    % reshape to Nx3
        %bound_box(occupied_grid_x,occupied_grid_y,occupied_grid_z) = 1;
        
        for k=1:size(keypoints,2)
            lineCoord = [keypoints(:,k); camera_center_position]';
            [ind_x, ind_y, ind_z] = wooRaytrace(double(grid_size), double(grid_bounds), double(lineCoord));
            indexes = [ind_x, ind_y, ind_z];        % Nx3
            
            % check intersections       
            % C:identical rows, C=A(ia,:) and C=B(ib,:).
            [C, ia, ib] = intersect(occupied_grid, indexes);
            intersected_rows = size(C,1);
            if intersected_rows >= 3
                is_keypoint_blocked(k) = 1;
            end
        end         
        
    end
end