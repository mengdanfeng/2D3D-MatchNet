function [track_point_idx, track_list] = get_tracks_list(point_to_uv_correspondences, track_length)

% get the list of image tracks
track_list = [];
for pointID=1:size(point_to_uv_correspondences,1)
    
    if size(point_to_uv_correspondences{pointID}, 1) > track_length
        
        for j=1:size(point_to_uv_correspondences{pointID}, 1)
            track_list = [track_list; pointID,  point_to_uv_correspondences{pointID}(j, :)];
        end
    end
    
end

% get the pointIDs for the tracks
if isempty(track_list)
    track_point_idx = [];
    return;
end
track_point_idx = unique(track_list(:,1));