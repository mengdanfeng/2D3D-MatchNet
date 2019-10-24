function Extract_SIFT_ISS_Area(track_list, track_point_idx, images, cam_id, ...
    submap_keypoint_idx_in_pcl, submap_pointcloud,volume_radius,...
    cam_global_poses, laser_global_poses, camera_laser_idx, cam_idx,G_camera_image, G_camera_ins, camera_intrinsics,...
    submap_sift_patch_dir, submap_iss_volume_dir,submap_proj_dir)
% cam_id: 1(left) or 2(right camera)

%% test
% track_points = (submap_keypoints(:,track_point_idx))';
min_half_size = 16;
max_half_size = 128;
%patch_size = 128;
min_point_in_volume = 100;

% get intrinsic params
fx = camera_intrinsics(1,1);
fy = camera_intrinsics(2,2);
cx = camera_intrinsics(1,3);
cy = camera_intrinsics(2,3);

first_camId = cam_idx(1); 

% extract image patch
%track_point_reproj = zeros(size(track_list,1),2);
submap_proj = cell(length(images),1);       % number of images in submap
for image_id=1:length(images)
    % plot projection of volume
    cur_frame_pose= cam_global_poses{cam_idx(image_id)};
    rel_pose= cur_frame_pose \ laser_global_poses{camera_laser_idx(first_camId)};
    transformed_pointcloud= rel_pose * [submap_pointcloud; ones(1, size(submap_pointcloud,2))];
    
    %Transform pointcloud into camera image frame
    xyz = (G_camera_image \ G_camera_ins * transformed_pointcloud).';
    xyz(:,4) = [];
    %Project points into image
    %uv = [ fx .* xyz(:,1) ./ xyz(:,3) + cx, fy .* xyz(:,2) ./ xyz(:,3) + cy];
    
    submap_proj{image_id} = xyz;
end 

% index of track point in full pointcloud
track_point_idx_in_pcl = submap_keypoint_idx_in_pcl(track_point_idx);
submap_pointcloud = submap_pointcloud';       % Nx3
track_point_in_pcl = submap_pointcloud(track_point_idx_in_pcl,:);

% compute pixel-wise distance
dist = pdist2(track_point_in_pcl, submap_pointcloud);
track_point_volumes = cell(length(track_point_idx),1);
track_point_volume_idx = cell(length(track_point_idx),1);
%track_point_volume_proj = cell(length(track_point_idx),1);
track_point_volume_num = zeros(length(track_point_idx),1);
%volume_center_idx = ones(length(track_point_idx),1);
for i=1:size(track_point_in_pcl, 1)    
    in_volume_index = find(dist(i,:)<=volume_radius);    
    track_point_volume_idx{i} = in_volume_index;
    %volume_center_idx(i) = find(in_volume_index == track_point_idx_in_pcl(i));
    %in_volume_num = length(in_volume_index);
    track_point_volume_num(i) = length(track_point_volume_idx{i});
    track_point_volumes{i} = submap_pointcloud(in_volume_index,:);
end

track_list_volume_idx = cell(size(track_list,1),1);
track_list_volume_num = zeros(size(track_list,1),1);
track_list_volume = cell(size(track_list,1),1);
%track_list_volume_center = zeros(size(track_list,1),1);
for i=1:length(track_point_idx)
    idx = find(track_list(:,1) == track_point_idx(i)); 
    track_list_volume_num(idx) = track_point_volume_num(i);
    for j=1:length(idx)
        track_list_volume_idx{idx(j),:} = track_point_volume_idx{i};
        track_list_volume{idx(j),:} = track_point_volumes{i};
        %track_list_volume_center(idx(j),:) = volume_center_idx(i);
    end
end


for i=1:size(track_list, 1)
    % extract 3d iss volume
    track_point_id = track_list(i,1);
    track_point_vol_num = track_list_volume_num(i);
    if(track_point_vol_num <min_point_in_volume)
        continue;
    end
    track_point_vol = track_list_volume{i};
    
    image_id = track_list(i,2);
    image = images{image_id};
    image_size = size(image);   % Attention: order of height/width
    sift_uv = round(track_list(i,5:6));
    xyz=submap_proj{image_id}(track_list_volume_idx{track_point_id},:);
    
    uv = [ fx .* xyz(:,1) ./ xyz(:,3) + cx, fy .* xyz(:,2) ./ xyz(:,3) + cy];

    
%     % iss volume too few points
%     track_point_vol = track_list_volume{i};     % Nx3
%     track_point_id = track_list(i,1);
%     if(size(track_point_vol,1)<min_point_in_volume)
%         continue;
%     end
  
    % visualize patch and projected iss volume
    figure(1),
    imshow(image);
    hold on
    % plot patch
    plot(sift_uv(1),sift_uv(2), 'r*');    
    hold on
    
%     % plot projection of volume
%     cur_frame_pose= cam_global_poses{cam_idx(image_id)};
%     rel_pose= cur_frame_pose \ laser_global_poses{camera_laser_idx(first_camId)};
%     transformed_pointcloud= rel_pose * [track_point_vol'; ones(1, size(track_point_vol',2))];
%     
%     %Transform pointcloud into camera image frame
%     xyz = (G_camera_image \ G_camera_ins * transformed_pointcloud).';
%     xyz(:,4) = [];
%     %Project points into image
%     uv = [ fx .* xyz(:,1) ./ xyz(:,3) + cx, fy .* xyz(:,2) ./ xyz(:,3) + cy];
    
    %Get indices of points that are projected into image
    in_front = xyz(:,3) >= 0;             
    in_img = (uv(:,1) >= 0.5 & uv(:,1) < size(image,2)-0.5) & (uv(:,2) >= 0.5 & uv(:,2) < size(image,1)-0.5);
    logical_idx= and(in_front,in_img);
    reproj_point_img_idx = find(logical_idx==1);
% 
%     if sum(reproj_point_img_idx) == 0
%         warning('No points project into image. Is the vehicle stationary?');
%     elseif sum(reproj_point_img_idx) < 1000
%         warning('Very few points project into image. Is the vehicle stationary?');
%     end

    % get reprojected point's uv coordinates
    uv_copy = uv;
    uv = uv(reproj_point_img_idx,:);    % Nx2
    
    % boundary of reprojection
    bound_x = [min(uv(:,1)), max(uv(:,1))];
    bound_y = [min(uv(:,2)), max(uv(:,2))];
    max_width = max(abs(bound_x-sift_uv(1)));
    max_height = max(abs(bound_y-sift_uv(2)));
    half_size = max(max_width, max_height);

    if half_size<min_half_size
        half_size = min_half_size;
    elseif half_size>max_half_size
        half_size = max_half_size;
    end

    top_left_x = int16(sift_uv(1) - half_size);
    top_left_y = int16(sift_uv(2) - half_size);
    bottom_right_x = int16(sift_uv(1) + half_size);
    bottom_right_y = int16(sift_uv(2) + half_size);
    if(top_left_x < 1 || top_left_y < 1 || bottom_right_x > image_size(1) || bottom_right_y > image_size(2))
        %fprintf(' out of range.\n');
        continue;
    end
    rectangle('Position', [top_left_x, top_left_y, 2*half_size, 2*half_size],'EdgeColor','r');   
        
    %DISPLAY
    colours = xyz(reproj_point_img_idx, 3);      
    scatter(uv(:,1),uv(:,2), 10, colours, '.');     
%     hold on
%     center_uv = uv_copy(track_list_volume_center(i),:);
%     %center_uv = uv(track_list_volume_center(i),:);
%     plot(center_uv(1),center_uv(2), 'r--o');
%     track_point_reproj(i,:) = center_uv;
    pause(0.1);
    hold off    
%     fprintf('   %d, img %d, cam %d: track_reproj: (%03f, %03f), calculated_reproj: (%03f, %03f)\n', ...
%             i, image_id, cam_id, ...
%             track_list(i,3), track_list(i,4),center_uv(1),center_uv(2));
%         
%     %dir_path = sprintf('%s%03d',sift_patch_dir, submap_id);
%     saveas(gcf, sprintf('%s/cam%d_%03d_%05d.png',submap_proj_dir, cam_id, image_id, i));
    
    % write patch
%     sift_patch = image(top_left_y:bottom_right_y,top_left_x:bottom_right_x, :);
%     patch_path = sprintf('%s/cam%d_%03d_%04d_%05d.png', submap_sift_patch_dir, cam_id, track_point_id,image_id, i);
%     imwrite(sift_patch,patch_path);
%     
%     % write 3d volume
%     volume_path = sprintf('%s/cam%d_%03d_%04d_%05d.pcd', submap_iss_volume_dir,cam_id, track_point_id,image_id,i);
%     pcwrite(pointCloud(track_point_vol), volume_path,'Encoding','ascii');   

end




