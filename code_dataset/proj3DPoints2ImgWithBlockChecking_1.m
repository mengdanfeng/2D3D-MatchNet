% project 3d points into images
% reproj_point_img_idx_list{imgID} = pointcloud_idx
% reproj_point_img_uv_list{imgID} = uv_coordinates_on_image

function [reproj_point_img_idx_list, reproj_point_img_uv_list] = ...
    proj3DPoints2ImgWithBlockChecking_1(images, cam_idx, camera_laser_idx, cam_global_poses, ...
    laser_global_poses, G_camera_image, G_camera_ins, pointcloud, keypoints, ...
    camera_intrinsics,camID, grid_resolution,submapID)

% get intrinsic params
fx = camera_intrinsics(1,1);
fy = camera_intrinsics(2,2);
cx = camera_intrinsics(1,3);
cy = camera_intrinsics(2,3);

first_camId = cam_idx(1); 
    
reproj_point_img_idx_list=cell(length(cam_idx), 1);
reproj_point_img_uv_list=cell(length(cam_idx), 1);
for imgID=1:length(cam_idx)
    fprintf(' progress: submapID %d, imageID %d/%d\n', submapID, imgID, length(cam_idx));
    camId = cam_idx(imgID);
    image = images{imgID};

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
    
    if sum(reproj_point_img_idx) == 0
        warning('No points project into image. Is the vehicle stationary?');
    elseif sum(reproj_point_img_idx) < 1000
        warning('Very few points project into image. Is the vehicle stationary?');
    end    
    
    % get reprojected point's uv coordinates
    uv1 = uv(reproj_point_img_idx,:);    
    colours = xyz(reproj_point_img_idx, 3);
%             
    %DISPLAY
    figure(1),
    imshow(image);
    colormap jet;
    hold on;
    scatter(uv1(:,1),uv1(:,2), 90, colours, '.');       
    hold off
    path=sprintf('/media/mengdan/data2/robotcar/grasshopper/lms_front_proj/gps_60m/2014-06-26-09-53-12/cam%d_bef_%02d_%03d.png',camID,submapID, imgID);
    saveas(gcf, path);
    
    
    %% check the 3D point is blocked or not
    grid_resolution_x = grid_resolution.x;        % 0.2
    grid_resolution_y = grid_resolution.y;
    grid_resolution_z = grid_resolution.z;
    
    camera_center_pose = rel_pose \ G_camera_ins;
    camera_center_position = camera_center_pose(1:3,4);
    
    reproj_in_img_points = pointcloud(:,reproj_point_img_idx);      % 3xN
    
    % visualize
    %figure,pcshow(pointCloud(reproj_in_img_points'));
%     figure(2),pcshow(pointCloud(reproj_in_img_points'));
%     hold on
%     scatter3(camera_center_position(1),camera_center_position(2),camera_center_position(3), 'filled');
%     view(0,90)
%     hold off

    % pcdenoise (optional)
    %submap_in_img_points_denoised = pcdenoise(pointCloud(submap_in_img_points'));
%     [reproj_in_img_points_denoised,inlierIndices,outlierIndices] = pcdenoise(pointCloud(reproj_in_img_points'));
%     reproj_in_img_points_denoised = (reproj_in_img_points_denoised.Location)';     % 3XN
%     reproj_point_img_idx = reproj_point_img_idx(inlierIndices);
%     figure,pcshow(pointCloud(reproj_in_img_points_denoised'));
    
    reproj_in_img_points_denoised = reproj_in_img_points;
    % grid params
    min_x = min(reproj_in_img_points_denoised(1,:));
    min_y = min(reproj_in_img_points_denoised(2,:));
    min_z = min(reproj_in_img_points_denoised(3,:));
    max_x = max(reproj_in_img_points_denoised(1,:));
    max_y = max(reproj_in_img_points_denoised(2,:));
    max_z = max(reproj_in_img_points_denoised(3,:));

    %[min_x, min_y, min_z] = min(submap_in_img_points,[],2);
    bound_x_min = floor(min_x/grid_resolution_x) * grid_resolution_x;
    bound_y_min = floor(min_y/grid_resolution_y) * grid_resolution_y;
    bound_z_min = floor(min_z/grid_resolution_z) * grid_resolution_z; 
    bound_x_max = ceil(max_x/grid_resolution_x) * grid_resolution_x;
    bound_y_max = ceil(max_y/grid_resolution_y) * grid_resolution_y;
    bound_z_max = ceil(max_z/grid_resolution_z) * grid_resolution_z;
    
    grid_bounds = [bound_x_min,bound_y_min,bound_z_min,bound_x_max,bound_y_max,bound_z_max];      
    grid_size_x = (bound_x_max - bound_x_min)/grid_resolution_x;
    grid_size_y = (bound_y_max - bound_y_min)/grid_resolution_y;
    grid_size_z = (bound_z_max - bound_z_min)/grid_resolution_z;
    grid_size = [grid_size_x, grid_size_y, grid_size_z];

    % plot cube
%     % plotCube()
%     x_length = bound_x_max - bound_x_min;
%     y_length = bound_y_max - bound_y_min;
%     z_length = bound_z_max - bound_z_min;
    
    % line segment, start
    %keypoints_idx = reproj_point_img_idx;
    %reproj_in_img_points_denoised;
    %keypoints = pointcloud(:,reproj_point_img_idx);
    %is_3Dpoint_blocked = zeros(size(reproj_in_img_points_denoised,2),1);

    % keypoints to grid position
    keypoints_in_image_idx = find(keypoints(1,:)>=bound_x_min&&keypoints(1,:)<=bound_x_max && ...
         keypoints(2,:)>=bound_y_min &&keypoints(2,:)<=bound_y_max && ...
         keypoints(3,:)>=bound_z_min &&keypoints(3,:)<=bound_z_max);
    keypoints_in_image = keypoints(:,keypoints_in_image_idx);
    keypoints_grid_x = floor((keypoints_in_image(1,:)-bound_x_min)./grid_resolution_x) + 1;
    keypoints_grid_y = floor((keypoints_in_image(2,:)-bound_y_min)./grid_resolution_y) + 1;
    keypoints_grid_z = floor((keypoints_in_image(3,:)-bound_z_min)./grid_resolution_z) + 1;
    keypoints_grid = [keypoints_grid_x; keypoints_grid_y; keypoints_grid_z]';    % 3xN

    intersected_rows = ones(size(keypoints_in_image,2),1);

    for k=1:size(keypoints_in_image,2)
        if(mod(k,100) ==0)
            fprintf('  progress: %d/%d\n',k,size(keypoints_in_image,2));
        end
        lineCoord = [keypoints_in_image(:,k); camera_center_position]';
        [ind_x, ind_y, ind_z] = wooRaytrace(double(grid_size), double(grid_bounds), double(lineCoord));
        indexes = double([ind_x, ind_y, ind_z]);        % Nx3
        
%%      3d visualization        
%         % 3d plot
%         figure(3);
%         ha1 = subplot(1,1,1);
%         axes(ha1);
%         hold on
%         xlabel('X');
%         ylabel('Y');
%         zlabel('Z');
%         view(45,45);
%         box on;
%         grid on;
%         for nVoxel = 1:size(indexes,1)
%             %[dy, dx, dz] = ind2sub([8 8 8],indexes(nVoxel,1));
%             dx = indexes(nVoxel,1);
%             dy = indexes(nVoxel,2);
%             dz = indexes(nVoxel,3);
%             vx = bound_x_min + [(dx-1)*grid_resolution_x (dx)*grid_resolution_x];
%             vy = bound_y_min + [(dy-1)*grid_resolution_y (dy)*grid_resolution_y];
%             vz = bound_z_min + [(dz-1)*grid_resolution_z (dz)*grid_resolution_z];
%             %disp([vx; vy; vz])
%             fv.vertices = [[vx(1) vy(1) vz(1)];[vx(2) vy(1) vz(1)];[vx(2) vy(2) vz(1)];[vx(1) vy(2) vz(1)]; ...
%                 [vx(1) vy(1) vz(2)];[vx(2) vy(1) vz(2)];[vx(2) vy(2) vz(2)];[vx(1) vy(2) vz(2)]];
%             fv.faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
%             h = patch(fv, 'FaceColor', 'blue', 'EdgeColor', 'blue');
%             tmp_point = reproj_in_img_points_denoised(:,k);
%             tmp_min_x = min(tmp_point(1),camera_center_position(1))-1;
%             tmp_max_x = max(tmp_point(1),camera_center_position(1))+1;
%             tmp_min_y = min(tmp_point(2),camera_center_position(2))-1;
%             tmp_max_y = max(tmp_point(2),camera_center_position(2))+1;
%             tmp_min_z = min(tmp_point(3),camera_center_position(3))-1;
%             tmp_max_z = max(tmp_point(3),camera_center_position(3))+1;
%             set(ha1,'XLim',[tmp_min_x tmp_max_x],'YLim',[tmp_min_y tmp_max_y],'ZLim',[tmp_min_z tmp_max_z],...
%                 'XTick',tmp_min_x:tmp_max_x,'YTick',tmp_min_y:tmp_max_y,'ZTick',tmp_min_z:tmp_max_z);
%             %set(ha1,'XLim',[min_x max_x],'YLim',[min_y max_y],'ZLim',[min_z max_z],'XTick',min_x:max_x,'YTick',min_y:max_y,'ZTick',min_z:max_z);
%             h.FaceAlpha = 0.2;
%         end
%         plot3([lineCoord(1) lineCoord(4)]',[lineCoord(2) lineCoord(5)]',[lineCoord(3) lineCoord(6)]','-r*');
%         hold on
%         scatter3(camera_center_position(1),camera_center_position(2),camera_center_position(3),'filled');
%         hold on, plotCube([x_length y_length z_length],[bound_x_min bound_y_min bound_z_min],.2,[1 0 0]);
%         pcshow(pointCloud(reproj_in_img_points'));
        % check intersections       
        % C:identical rows, C=A(ia,:) and C=B(ib,:).
        %fprintf(' occupied_grid rows: %d, grid_rows: %d\n', size(occupied_grid,1),size(indexes,1));
        [C, ~, ~] = intersect(keypoints_grid, single(indexes(10:end,:)),'rows');
        intersected_rows(k,1) = size(C,1);
    end         
    
    %is_3Dpoint_blocked(intersected_rows>=3) = 1;
    valid_index = intersected_rows<=1;
    
    reproj_point_img_idx = keypoints_in_image_idx(valid_index);    
    reproj_point_img_idx_list{imgID} = reproj_point_img_idx;
    
    %fprintf('  bef block check:%d, aft: %d\n', size(reproj_in_img_points_denoised,2),length(reproj_point_img_idx));
    
    % get reprojected point's uv coordinates
    uv = uv(reproj_point_img_idx,:);
    reproj_point_img_uv_list{imgID} = uv;    
    
    colours = xyz(reproj_point_img_idx, 3);
            
%    DISPLAY
    figure(2),
    imshow(image);
    colormap jet;
    hold on;
    scatter(uv(:,1),uv(:,2), 90, colours, '.');       
    
    hold off
    path=sprintf('/media/mengdan/data2/robotcar/grasshopper/lms_front_proj/gps_60m/2014-06-26-09-53-12/cam%d_aft_%02d_%03d.png',camID,submapID, imgID);
    saveas(gcf, path);
    %pause
end