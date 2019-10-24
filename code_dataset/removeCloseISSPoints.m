function [new_track_point_idx, new_track_list] = ...
    removeCloseISSPoints(track_list, track_point_idx, submap_keypoints)
% remove iss points that are close to each other
% threshold dist: 5m

% track_point_idx: point id that are tracked
% track_list: [3d_pt_id, img_id, reproj_uv, sift_uv; ...], Nx6 array
% submap_keypoints: keypoints position, 3xN
    
    if isempty(track_list)
        new_track_point_idx = [];
        new_track_list = [];
        return;
    end
    
    track_point_num = length(track_point_idx);
    track_point_position = (submap_keypoints(:,track_point_idx))';  %Nx3        
    iss_dist = pdist2(track_point_position, track_point_position);
    
    delete_idx = zeros(track_point_num,1);    
    for i=1:track_point_num-1
        if delete_idx(i) == 0 
            id = find(iss_dist(i, (i+1):end) <= 4);
            delete_idx(i+id) = 1;
        end
    end
    delete_track_point_idx = track_point_idx(delete_idx==1);
    
    delete_list = zeros(size(track_list,1),1);
    for i = 1:length(delete_track_point_idx)
        id = (track_list(:,1) == delete_track_point_idx(i));
        delete_list(id) = 1;
    end
    
    new_track_list = track_list(delete_list==0,:);
    if isempty(new_track_list)
        new_track_point_idx = [];
        return;
    end
    new_track_point_idx = unique(track_list(:, 1));
    
        