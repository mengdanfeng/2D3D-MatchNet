function [camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeSmallCamPoses_2(camera_timestamps, camera_laser_idx, cam_global_poses, camera_reading_angle, camera_reading_distance)

% get relative transformation
cam_rel_poses_Norm = zeros(size(camera_timestamps,1), 2);

for i=2:size(camera_timestamps,1)
    T = cam_global_poses{i-1} \ cam_global_poses{i};
    [x1, y1, z1]= getEulerAngles(T(1:3,1:3));
    cam_rel_poses_Norm(i,:) = [norm([x1, y1, z1])*180/pi, norm(T(1:3,4))]; 
end

% remove camera poses with small movement
idx = find(cam_rel_poses_Norm(:,1) > camera_reading_angle | cam_rel_poses_Norm(:,2) > camera_reading_distance);
idx = [1; idx];
camera_timestamps = camera_timestamps(idx,:);
camera_laser_idx = camera_laser_idx(idx,:);
cam_global_poses = cam_global_poses(idx,1);