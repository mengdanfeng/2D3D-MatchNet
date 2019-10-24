function [track_high_curvature_point_idx, track_high_curvature_list] = ... 
    getHighCurvatureTracks(high_curvature_points, pointcloud_downsampled, track_point_idx, track_list)

% get nearest neighbors from downsampled point tracks
tracked_points = pointcloud_downsampled(:, track_point_idx);
[idx, dist] = knnsearch( tracked_points', high_curvature_points', 'dist','euclidean','k',1);
idx = idx(dist == 0);

track_high_curvature_point_idx = track_point_idx(idx);
idx_list = [];
for i=1:length(track_high_curvature_point_idx)

    idx = find(track_list(:,1) == track_high_curvature_point_idx(i));
    idx_list = [idx_list; idx];
end
track_high_curvature_list = track_list(idx_list, :);

% figure,
% scatter3(-off_groundplane_pointcloud(2,1:end), ...
%          -off_groundplane_pointcloud(1,1:end),...
%          -off_groundplane_pointcloud(3,1:end), 1, normalized_curvature(1:end), '.');
% hold on
% scatter3(-off_groundplane_pointcloud(2,pointcloud_high_curvature_idx), ...
%          -off_groundplane_pointcloud(1,pointcloud_high_curvature_idx),...
%          -off_groundplane_pointcloud(3,pointcloud_high_curvature_idx), 5, 'r.');
% hold off
% axis equal