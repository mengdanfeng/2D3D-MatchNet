% get the laser scan idx for each submap
function submap_laser_idx = get_submap_laser_indices(submap_idx, camera_laser_idx)

nSubmaps = submap_idx(end);
submap_laser_idx = cell(nSubmaps, 1);
for i=1:nSubmaps
    
    % get camera idx in a submap
    cam_idx = find(submap_idx == i);
    
    for j=1:size(cam_idx, 1)
        
        % check idx diff -- get last 5 scans if diff is >5
        last_idx = 0;
        if ~isempty(submap_laser_idx{i}) last_idx = submap_laser_idx{i}(end); end
            
        d = camera_laser_idx(cam_idx(j)) - last_idx;
        r = last_idx+1 : camera_laser_idx(cam_idx(j));
        if d > 5
            r = camera_laser_idx(cam_idx(j))-4 : camera_laser_idx(cam_idx(j));
        end
     
        submap_laser_idx{i} = [submap_laser_idx{i}, r];
    end
    
end