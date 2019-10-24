function valid_keypoints_index = ExtractISSVolume(submap_keypoints, submap_pointclouds, submap_iss_volume_dir, ...
    volume_radius,submapID)
% this function extract iss volumes 

% 
%volume_radius = 1;
min_point_in_volume = 100;
submap_keypoints = submap_keypoints';
submap_pointclouds = submap_pointclouds';
dist = pdist2(submap_keypoints, submap_pointclouds); % should be Nx3

% remove close iss points
% delete_idx = zeros(size(submap_keypoints,1),1);    
% iss_dist = pdist2(submap_keypoints,submap_keypoints);
% for i=1:size(submap_keypoints,1)-1
%     if delete_idx(i) == 0 
%         id = find(iss_dist(i, (i+1):end) <= 2);
%         delete_idx(i+id) = 1;
%     end
% end
% submap_keypoints = submap_keypoints(delete_idx==0,:);

% 
iss_point_num = size(submap_keypoints, 1);
iss_point_volumes = {};
volume_paths = {};
iss_xyz = {};
iss_id = 0;
valid_keypoints_index = [];
for i=1:iss_point_num
    in_volume_index = find(dist(i,:)<=volume_radius);
    in_volume_points = submap_pointclouds(in_volume_index, :);
    in_volume_num = length(in_volume_index);
    
    % check volume point num
    if(in_volume_num<min_point_in_volume)
        continue;
    end
    
    valid_keypoints_index = [valid_keypoints_index, i];
    
    iss_id = iss_id + 1;
    
    % zero mean
    in_volume_points = in_volume_points - repmat(submap_keypoints(i,:),in_volume_num,1);
    
    % set path
    volume_path = sprintf('%s/%03d_%04d.pcd', submap_iss_volume_dir,submapID,iss_id);
    iss_point_volumes{iss_id} = pointCloud(in_volume_points);
    volume_paths{iss_id} = volume_path;
    iss_xyz{iss_id} = {iss_id, volume_path, submap_keypoints(i,:)};
end

% return updated keypoints

% save iss volume
cellfun(@pcwrite, iss_point_volumes, volume_paths);

% save iss keypoint's xyz
iss_xyz_path = sprintf('%s/%03d.mat', submap_iss_volume_dir,submapID);
save(iss_xyz_path, 'iss_xyz');
