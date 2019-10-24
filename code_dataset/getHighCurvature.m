% set indicator for points with high curvature
% point ids of points with high curvature --> pointcloud_high_curvature_idx
% pointcloud_curvature_indicator{pointID} = 0 (low curvature), 1 (high curvature)

function [pointcloud_high_curvature_idx, pointcloud_curvature_indicator] = ...
    getHighCurvature(pointcloud, normalized_curvature, curvatureThresh)


pointcloud_curvature_indicator = zeros(size(pointcloud,2), 1);
pointcloud_high_curvature_idx = normalized_curvature > curvatureThresh;
pointcloud_curvature_indicator(pointcloud_high_curvature_idx) = 1;