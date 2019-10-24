function [camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps2(camera_timestamps, laser_timestamps)

% find the intersection of timestamps among camera1, camera2 and laser
% synchronize cameras (make both cameras same number of timestamps)
% camera 1 is the reference
% take the bigger 


% camera 1 compare with laser timestamps, find the intersection
cam_lower_idx = 1; 
if camera_timestamps(1,1) < laser_timestamps(1,1)
    cam_lower_idx = find(camera_timestamps(:,1)>=laser_timestamps(1,1), 1, 'first');
end

cam_upper_idx = size(camera_timestamps,1);
if camera_timestamps(end,1) > laser_timestamps(end,1)
    cam_upper_idx = find(camera_timestamps(:,1)<=laser_timestamps(end,1), 1, 'last');
end

range=cam_lower_idx:cam_upper_idx;
% camera1_timestamps = zeros(size(range,2),2);

camera_timestamps = camera_timestamps(range,:);

% for every camera timestamp, find the closest laser timestamp
fprintf(' camera and laser diff: %d (should be positive)\n',camera_timestamps(1,1) - laser_timestamps(1,1));
camera_laser_idx = zeros(size(camera_timestamps,1),1);
for i=1:size(camera_timestamps,1)
   [val, idx] = min(abs(camera_timestamps(i,1) - laser_timestamps(:,1)));
   camera_laser_idx(i,1) = idx;
end



% remove first nFirstLaserRemove laser scans
% laser_timestamps = laser_timestamps(nFirstLaserRemove+1:end,:);
% idx = find(camera_laser_idx > nFirstLaserRemove);
% camera_laser_idx = camera_laser_idx(idx)-nFirstLaserRemove;
% camera_timestamps{1} = camera_timestamps{1}(idx,:);
% camera_timestamps{2} = camera_timestamps{2}(idx,:);

% remove the last nLastLaserRemove laser scans
% laser_timestamps = laser_timestamps(1:end-nLastLaserRemove,:);
% idx = find(camera_laser_idx < size(laser_timestamps,1));
% camera_laser_idx = camera_laser_idx(idx);
% camera_timestamps{1} = camera_timestamps{1}(idx,:);
% camera_timestamps{2} = camera_timestamps{2}(idx,:);

