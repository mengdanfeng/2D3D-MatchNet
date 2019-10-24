% project 3d points into images
% reproj_point_img_idx_list{imgID} = pointcloud_idx
% reproj_point_img_uv_list{imgID} = uv_coordinates_on_image

function [reproj_point_img_idx_list, reproj_point_img_uv_list] = ...
    projNeighbor3DPoints2Img(images, cam_idx, camera_laser_idx, cam_global_poses, ...
    laser_global_poses, G_camera_image, G_camera_ins, pointcloud, camera_intrinsics,camID,...
    point_laser_idx,submapID,pcl_proj_dir)

% get intrinsic params
fx = camera_intrinsics(1,1);
fy = camera_intrinsics(2,2);
cx = camera_intrinsics(1,3);
cy = camera_intrinsics(2,3);

first_camId = cam_idx(1); 
    
reproj_point_img_idx_list=cell(length(cam_idx), 1);
reproj_point_img_uv_list=cell(length(cam_idx), 1);


%inv_initial_laser_global_pose = laser_global_poses{camera_laser_idx(first_camId)};
for imgID=1:length(cam_idx)
    
    %imgID = 29;
    
    camId = cam_idx(imgID);
    image= images{imgID};
    
    % for every camera frame, find the closest +-80 laser frames
    neighbor_laser_scan_idx = [max((camera_laser_idx(camId)-80),1):min((camera_laser_idx(camId)+80), length(laser_global_poses))];
    neighbor_point_idx = [];
    for i=1:length(neighbor_laser_scan_idx)
        tmp_idx = find(point_laser_idx == neighbor_laser_scan_idx(i));
        neighbor_point_idx = [neighbor_point_idx,tmp_idx];
    end
    neighbor_pointcloud = pointcloud(:,neighbor_point_idx);    
    
    
    % transform point cloud to current image frame
    cur_frame_pose= cam_global_poses{camId};
    rel_pose= cur_frame_pose \ laser_global_poses{camera_laser_idx(first_camId)};
    transformed_pointcloud= rel_pose * [neighbor_pointcloud; ones(1, size(neighbor_pointcloud,2))];
    
    %Transform pointcloud into camera image frame
    xyz = (G_camera_image \ G_camera_ins * transformed_pointcloud).';
    xyz(:,4) = [];

    %Project points into image
    uv = [ fx .* xyz(:,1) ./ xyz(:,3) + cx, fy .* xyz(:,2) ./ xyz(:,3) + cy];

    %Get indices of points that are projected into image
    in_front = xyz(:,3) >= 0;             
    in_img = (uv(:,1) >= 0.5 & uv(:,1) < size(image,2)-0.5) & (uv(:,2) >= 0.5 & uv(:,2) < size(image,1)-0.5);
    logical_idx= and(in_front,in_img);
    reproj_point_img_idx = find(logical_idx==1);
    
    reproj_point_img_idx_list{imgID} = neighbor_point_idx(reproj_point_img_idx);

    if isempty(reproj_point_img_idx)
        warning('No points project into image. Is the vehicle stationary?');
    elseif length(reproj_point_img_idx) < 200
        warning('Very few points project into image. Is the vehicle stationary?');
    end

    % get reprojected point's uv coordinates
    uv = uv(reproj_point_img_idx,:);
    reproj_point_img_uv_list{imgID} = uv;
    
    
%     colours = xyz(reproj_point_img_idx, 3);
%             
    %DISPLAY
%     figure(1),
%     imshow(image);
%     colormap jet;
%     hold on;
%     scatter(uv(:,1),uv(:,2), 10, colours, '.');       
%     
%     hold off
%     path=sprintf('%skeypts_cam%d_%02d_%03d.png',pcl_proj_dir,camID,submapID,imgID);
%     %saveas(gcf, path);
%     pause(0.1);
end