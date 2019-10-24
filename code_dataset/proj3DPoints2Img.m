% project 3d points into images
% reproj_point_img_idx_list{imgID} = pointcloud_idx
% reproj_point_img_uv_list{imgID} = uv_coordinates_on_image

function [reproj_point_img_idx_list, reproj_point_img_uv_list] = ...
    proj3DPoints2Img(images, cam_idx, camera_laser_idx, cam_global_poses, ...
    laser_global_poses, G_camera_image, G_camera_ins, pointcloud, camera_intrinsics,camID)

% get intrinsic params
fx = camera_intrinsics(1,1);
fy = camera_intrinsics(2,2);
cx = camera_intrinsics(1,3);
cy = camera_intrinsics(2,3);

first_camId = cam_idx(1); 
    
reproj_point_img_idx_list=cell(length(cam_idx), 1);
reproj_point_img_uv_list=cell(length(cam_idx), 1);
for imgID=1:length(cam_idx)
    

    camId = cam_idx(imgID);
    image= images{imgID};

    % transform point cloud into first laser frame
    cur_frame_pose= cam_global_poses{camId};
    rel_pose= cur_frame_pose \ laser_global_poses{camera_laser_idx(first_camId)};
    transformed_pointcloud= rel_pose * [pointcloud; ones(1, size(pointcloud,2))];
    
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
    
    reproj_point_img_idx_list{imgID} = reproj_point_img_idx;

    if sum(reproj_point_img_idx) == 0
        warning('No points project into image. Is the vehicle stationary?');
    elseif sum(reproj_point_img_idx) < 1000
        warning('Very few points project into image. Is the vehicle stationary?');
    end

    % get reprojected point's uv coordinates
    uv = uv(reproj_point_img_idx,:);
    reproj_point_img_uv_list{imgID} = uv;
    
    colours = xyz(reproj_point_img_idx, 3);
%             
    %DISPLAY
    figure(1),
    imshow(image);
    colormap jet;
    hold on;
    scatter(uv(:,1),uv(:,2), 90, colours, '.');       
    
    hold off
    path=sprintf('/media/mengdan/data2/robotcar/grasshopper/lms_front_proj/gps_60m/2014-06-26-09-53-12/cam%d_001_%03d.png',camID,imgID);
    saveas(gcf, path);
    %pause
end