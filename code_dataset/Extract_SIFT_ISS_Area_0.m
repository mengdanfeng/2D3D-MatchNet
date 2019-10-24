function Extract_SIFT_ISS_Area_0(track_list, track_point_idx, images, patch_width, patch_height, submap_id, sift_patch_dir, cam_id, ...
    submap_keypoint_idx_in_pcl, pointcloud, volume_radius, grid_resolution)
% cam_id: 1(left) or 2(right camera)

% index of track point in full pointcloud
valid_point_idx = submap_keypoint_idx_in_pcl(track_point_idx);

%% check the 3D point is blocked or not
grid_resolution_x = grid_resolution.x;        % 0.2
grid_resolution_y = grid_resolution.y;
grid_resolution_z = grid_resolution.z;

% grid params
min_x = min(pointcloud(1,:));
min_y = min(pointcloud(2,:));
min_z = min(pointcloud(3,:));
max_x = max(pointcloud(1,:));
max_y = max(pointcloud(2,:));
max_z = max(pointcloud(3,:));

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
grid = cell(grid_size);
for i=1:grid_size_x
    for j=1:grid_size_y
        for k=1:grid_size_z
            grid{i,j,k} = [];
        end
    end
end
            

occupied_grid_x = floor((pointcloud(1,:)-bound_x_min)./grid_resolution_x) + 1;
occupied_grid_y = floor((pointcloud(2,:)-bound_y_min)./grid_resolution_y) + 1;
occupied_grid_z = floor((pointcloud(3,:)-bound_z_min)./grid_resolution_z) + 1;
occupied_grid = [occupied_grid_x; occupied_grid_y; occupied_grid_z]';    % 3xN

for i=1:size(pointcloud,2)
    grid{occupied_grid(:,i)} = [grid{occupied_grid(:,i)},pointcloud(:,i)];
end


for i=1:valid_point_idx
    point_in_volume = [];
    point_grid_id = occupied_grid(submap_keypoint_idx_in_pcl(i));
    volume_grid_x_start = max(point_grid_id(1) - volume_radius, 1);
    volume_grid_x_end = min(point_grid_id(1) + volume_radius, grid_size_x);
    volume_grid_y_start = max(point_grid_id(2) - volume_radius, 1);
    volume_grid_y_end = min(point_grid_id(2) + volume_radius, grid_size_y);
    volume_grid_z_start = max(point_grid_id(3) - volume_radius, 1);
    volume_grid_z_end = min(point_grid_id(3) + volume_radius, grid_size_z);
    for xx = volume_grid_x_start : volume_grid_x_end
        for yy = volume_grid_y_start : volume_grid_y_end
            for zz = volume_grid_z_start : volume_grid_z_end
                dist = norm([xx, yy, zz] - point_grid_id);
                if dist > volume_radius
                    continue;
                end
                
                point_in_volume = [point_in_volume, grid{xx,yy,zz}];
               
                
            end
        end
    end
    
    if(size(point_in_volume,2)<min_point_in_volume)
        continue;
    end
    
    pcwrite(pointCloud(point_in_volume), 'xxxxx');
end


% keypoints to grid position
%bounding_box = zeros(grid_size_x, grid_size_y, grid_size_z);




% extract image patch
for i=1:size(track_list, 1)
    image_id = track_list(i,2);
    image = images(image_id);
    image_size = size(image);   % Attention: order of height/width
    sift_uv = round(track_list(i,5:6));
    
    % sift point out of range
    if (sift_uv(1) < patch_width/2 || sift_uv(1) > image_size(2)-patch_width/2 ...
       || sift_uv(2) < patch_height/2 || sift_uv(2) > image_size(1)-patch_height/2)
        continue;
    end
    
    patch_column = (sift_uv(2)-patch_height/2+1):(sift_uv(2)+patch_height/2);
    patch_row = (sift_uv(1)-patch_width/2+1):(sift_uv(1)+patch_width/2);
    sift_patch = image(patch_column, patch_row, :);
    
    % write patch
    patch_path = sprintf('%s%s/cam%d_%03d_%05d.png', sift_patch_dir, num2str(submap_id),cam_id, image_id, i);
    imwrite(sift_patch,patch_path);
    
    
    % iss volume
    iss_id = submap_keypoint_idx_in_pcl(track_list(i,1));
    
   

end




