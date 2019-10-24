function [laser_timestamps, camera_timestamps, camera_laser_idx, cam_global_poses, laser_global_poses] = ...
    removeSmallLaserPoses(laser_timestamps, camera_timestamps, laser_global_poses, ...
    laser_reading_angle, laser_reading_distance, ...
    camera_laser_idx,cam_global_poses)
% Function: remove laser_timestamps with small motion
% Input: 
%   -- laser_timestmaps: all laser_timestmaps
%   -- laser_global_poses: pose of each laser_timestamps
%   -- laser_reading_angle: minimum travel angle
%   -- laser_reading_distance: minimum travel distance
% Output:
%   -- laser_timestamps: filtered laser_timestamps
%   -- laser_global_poses: pose of filtered laser_timestamps
%   -- camera_laser_idx: for each camera, the nearest laser_timestamp idx

% get relative transformation
laser_rel_poses_Norm = zeros(size(laser_timestamps,1),2);

for i=2:size(laser_timestamps,1)
    T = laser_global_poses{i-1} \ laser_global_poses{i};
    [x1, y1, z1]= getEulerAngles(T(1:3,1:3));
    laser_rel_poses_Norm(i,:) = [norm([x1, y1, z1])*180/pi, norm(T(1:3,4))]; 
end

% remove laser poses with small movement
idx = find(laser_rel_poses_Norm(:,1) > laser_reading_angle | laser_rel_poses_Norm(:,2) > laser_reading_distance);
idx = [1; idx];
laser_timestamps = laser_timestamps(idx,:);
laser_global_poses = laser_global_poses(idx,1);

[~, ia, ~] = intersect(camera_laser_idx, idx);
camera_laser_idx = camera_laser_idx(ia,:);
camera_timestamps{1} = camera_timestamps{1}(ia,:);
camera_timestamps{2} = camera_timestamps{2}(ia,:);
cam_global_poses{1} = cam_global_poses{1}(ia,1);
cam_global_poses{2} = cam_global_poses{2}(ia,1);


% % for every camera timestamp, find the closest laser timestamp
% camera_laser_idx = zeros(size(camera_timestamps{1},1),1);
% for i=1:size(camera_timestamps{1},1)
%    [~, idx] = min(abs(camera_timestamps{1}(i,1) - laser_timestamps(:,1)));
%    camera_laser_idx(i,1) = idx;
% end
