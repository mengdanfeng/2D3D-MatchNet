function [new_track_point_idx, new_track_list] = ...
    removeCloseSIFTPoints(track_list, img_num)
% track_point_idx: point id that are tracked
% track_list: [3d_pt_id, img_id, reproj_uv, sift_uv; ...], Nx6 array
    
    if isempty(track_list)
        new_track_point_idx = [];
        new_track_list = [];
        return;
    end
    
    img_sift = cell(img_num, 1);
    for i = 1 : img_num
        id = find(track_list(:,2)==i);
        img_sift{i} = [track_list(id,5:6),id];
    end
    
    delete_list = zeros(size(track_list, 1),1);
    for i = 1 : img_num
        sift_points = img_sift{i}(:,1:2);
        local_delete_list = zeros(size(sift_points, 1),1);
        dist = pdist2(sift_points, sift_points);
        for j = 1 : size(sift_points, 1) - 1
            if local_delete_list(j) == 0
                id = find(dist(j, (j+1) : end) <= 38);
                local_delete_list(id+j) = 1;
            end
        end
        
        if sum(local_delete_list==0) == size(sift_points, 1)
            continue;
        end
        
        sift_line_idx = img_sift{i}(:, 3);
        delete_list(sift_line_idx(local_delete_list == 1)) = 1;
        
    end
    
    new_track_list = track_list(delete_list == 0,:);
    if isempty(new_track_list)
        new_track_point_idx = [];
        return;
    end
    new_track_point_idx = unique(new_track_list(:, 1));
    
        