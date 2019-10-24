function [image, pointcloud, is_valid] = buildPCL(camera_id, camera_laser_idx, laser_frames_number,...
    laser_timestamps, laser_global_poses,laser_dir, G_ins_laser, camera_timestamps,...
    camera_intrinsics,cam_global_poses, G_camera_image,G_camera_ins,...
    camera_dir, LUT)

    % This function build pointcloud around image i.
    
    % find corresponding laser id 
    %start_laser_id = max(1, camera_laser_idx(camera_id) - laser_frames_number);
    start_laser_id = max(1, camera_laser_idx(camera_id));
    end_laser_id = min(length(laser_timestamps), start_laser_id + 8*laser_frames_number -1);
    laser_scan_num = end_laser_id - start_laser_id + 1;

    init_laser_global_pose = laser_global_poses{start_laser_id};
    
    pointcloud = [];
    
    is_valid = 1;

    for i=1:laser_scan_num
        laser_id = start_laser_id + i - 1;
        scan_path = [laser_dir '/' num2str(laser_timestamps(laser_id,1)) '.bin'];
        scan_file = fopen(scan_path);
        scan = fread(scan_file, 'double');
        fclose(scan_file);

        scan = reshape(scan, [3 numel(scan)/3]);
        scan(3,:) = zeros(1, size(scan,2));

        scan = init_laser_global_pose \ (laser_global_poses{laser_id} * G_ins_laser) * [scan; ones(1, size(scan,2))];
        pointcloud = [pointcloud scan(1:3,:)];
    end    
    
    % remove plane ground
    [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.6);
    pointcloud= pointcloud(:,out_plane);
    
%     figure,pcshow(pointCloud(pointcloud'));
%     axis off
%     grid off
    % read image
    image = readImage(camera_id, camera_timestamps, camera_dir, LUT);
    %figure,imshow(image)
    
    % project pcl to image
    proj_num = projectPCL2Img(image, camera_id, camera_intrinsics, pointcloud, cam_global_poses, ...
        G_camera_image, G_camera_ins, init_laser_global_pose);
    
    % save image and corresponding pcl
    if proj_num < 5000
        fprintf('  Image %d not enough projected points: %d\n', camera_id, proj_num);
        is_valid = 0;        
    end
        
end

function image = readImage(img_id, camera_timestamps, camera_dir, LUT)
    % this funciton read image 
    image = LoadImage(camera_dir, camera_timestamps(img_id,1), LUT);
    if isempty(image)
        fprintf('  image does not exist. dir:%s, time: %f\n', camera_dir,camera_timestamps(cam_idx(imgID)));
    end
end


function proj_num = projectPCL2Img(image, img_id,camera_intrinsics, pointcloud, cam_global_poses, ...
     G_camera_image, G_camera_ins, init_laser_global_pose)
    % this function projects pointcloud to each image
    % get intrinsic params
    fx = camera_intrinsics(1,1);
    fy = camera_intrinsics(2,2);
    cx = camera_intrinsics(1,3);
    cy = camera_intrinsics(2,3);

    % transform point cloud into first laser frame
    cur_frame_pose= cam_global_poses{img_id};
    rel_pose= cur_frame_pose \ init_laser_global_pose;
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
    
    proj_num = length(reproj_point_img_idx);

%     reproj_point_img_idx_list{img_id} = reproj_point_img_idx;
%     fprintf('   pcl: %d, 2d points: %d\n', size(pointcloud,2), length(reproj_point_img_idx));
%     if sum(reproj_point_img_idx) == 0
%         warning('No points project into image. Is the vehicle stationary?');
%     elseif sum(reproj_point_img_idx) < 1000
%         warning('Very few points project into image. Is the vehicle stationary?');
%     end
% 
%     % get reprojected point's uv coordinates
%     uv = uv(reproj_point_img_idx,:);
% %     reproj_point_img_uv_list{imgID} = uv;
% 
%     colours = xyz(reproj_point_img_idx, 3);
    %             
    %DISPLAY
%     figure(2),
%     imshow(image);
%     colormap jet;
%     hold on;
%     scatter(uv(:,1),uv(:,2), 90, colours, '.');    
%     hold off
%     
%     base_path = '/media/mengdan/data3/robotcar/bumblebee/projection/2014-06-26-09-53-12';
%     path = sprintf('%s/%05d.png', base_path, img_id);
%     saveas(gcf, path);
    
    
end
