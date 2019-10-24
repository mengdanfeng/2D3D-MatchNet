function [good_track_point_idx, good_track_list] = nonMaximalSuppression(pointcloud, normalized_curvature, track_point_idx, track_list, curvatureThresh)

% find nearest neighbors within a radius
track_points = pointcloud(: ,track_point_idx);
[idx, dist] = rangesearch(pointcloud',track_points',0.3);

% find number of neighbors in all points
nNeigh = zeros(size(idx,1), 1);
for i=1:size(idx,1)
    nNeigh(i) = size(idx{i},2);
end

% get max neighbor count
maxNeighCnt = max(nNeigh);

% do non-maximal suppression (get point tracks that have the highest curvature locally)
good_track_point_idx=zeros(size(track_point_idx));
cnt=1;
for i=1:size(idx,1)
    
    point_curvature =  normalized_curvature(track_point_idx(i));  
    max_neigh_curvature = max(normalized_curvature(idx{i}));
    
    if point_curvature == max_neigh_curvature && nNeigh(i) > 0.2*maxNeighCnt && point_curvature > curvatureThresh
        good_track_point_idx(cnt) = track_point_idx(i);
        cnt = cnt + 1;
    end
end
good_track_point_idx = good_track_point_idx(1:cnt-1);

% get good track list
good_track_list = [];
for i=1:length(good_track_point_idx)
    
    idx = track_list(:,1) == good_track_point_idx(i);
    good_track_list = [good_track_list; track_list(idx,:)];
end
