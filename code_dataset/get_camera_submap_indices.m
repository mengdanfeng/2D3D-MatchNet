% get submap indices
function submap_idx = get_camera_submap_indices(camera_timestamps, cam_global_poses, submap_cover_distance)

% get relative transformation
cam_rel_poses_Norm = zeros(size(camera_timestamps,1), 2);
for i=2:size(camera_timestamps,1)
    T = cam_global_poses{i-1} \ cam_global_poses{i};
    [x1, y1, z1]= getEulerAngles(T(1:3,1:3));
    cam_rel_poses_Norm(i,:) = [norm([x1, y1, z1])*180/pi, norm(T(1:3,4))]; 
end

% get submap indices
submap_idx=ones(size(camera_timestamps,1),1);
sum_dist=0; ref_idx=1; submap_counter=1;
for i=2:size(camera_timestamps,1)
    
    sum_dist = sum_dist + cam_rel_poses_Norm(i,2);
    % create submap if distance travel is more than the threshold
    if sum_dist > submap_cover_distance | i==size(camera_timestamps,1)
        
        submap_idx(ref_idx:i) = submap_counter;
        submap_counter=submap_counter+1;
        ref_idx = i+1; 
        sum_dist = 0;
    end
end