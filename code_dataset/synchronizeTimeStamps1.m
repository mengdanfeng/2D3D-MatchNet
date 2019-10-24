function [camera_timestamps, laser_timestamps, camera_laser_idx] = ...
    synchronizeTimeStamps1(camera_timestamps, laser_timestamps)

% find the intersection of timestamps among camera1, camera2 and laser
% synchronize cameras (make both cameras same number of timestamps)
% camera 1 is the reference
% take the bigger 
start_id = 1;   
if camera_timestamps{1}(1,1) < camera_timestamps{2}(1,1)
    [val, start_id] = min(abs(camera_timestamps{2}(1,1) - camera_timestamps{1}(:,1)));
end

end_id = size(camera_timestamps{1},1);
if camera_timestamps{1}(end,1) > camera_timestamps{2}(end,1)
    [val, end_id] = min(abs(camera_timestamps{1}(end,1) - camera_timestamps{2}(:,1)));
end

range=start_id:end_id;
camera1_timestamps = zeros(size(range,2),2);
camera2_timestamps = zeros(size(range,2),2);
for i=1:size(range,2)
    id = range(i);
    camera1_timestamps(i,:) = camera_timestamps{1}(id,:);
    [val, idx] = min(abs(camera1_timestamps(i,1) - camera_timestamps{2}(:,1)));
    camera2_timestamps(i,:) = camera_timestamps{2}(idx,:);
    
end
camera_timestamps{1} = camera1_timestamps;
camera_timestamps{2} = camera2_timestamps;

% camera 1 compare with laser timestamps, find the intersection
cam_lower_idx = 1; 
if camera_timestamps{1}(1,1) < laser_timestamps(1,1)
    cam_lower_idx = find(camera_timestamps{1}(:,1)>=laser_timestamps(1,1), 1, 'first');
end

cam_upper_idx = size(camera_timestamps{1},1);
if camera_timestamps{1}(end,1) > laser_timestamps(end,1)
    cam_upper_idx = find(camera_timestamps{1}(:,1)<=laser_timestamps(end,1), 1, 'last');
end

range=cam_lower_idx:cam_upper_idx;
camera1_timestamps = zeros(size(range,2),2);
camera2_timestamps = zeros(size(range,2),2);
for i=1:size(range,2)
    id = range(i);
    camera1_timestamps(i,:) = camera_timestamps{1}(id,:);
    [val, idx] = min(abs(camera1_timestamps(i,1) - camera_timestamps{2}(:,1)));
    camera2_timestamps(i,:) = camera_timestamps{2}(idx,:);
    
end
camera_timestamps{1} = camera1_timestamps;
camera_timestamps{2} = camera2_timestamps;

% for every camera timestamp, find the closest laser timestamp
fprintf(' camera and laser diff: %d (should be positive)\n',camera_timestamps{1}(1,1) - laser_timestamps(1,1));
camera_laser_idx = zeros(size(camera_timestamps{1},1),1);
for i=1:size(camera_timestamps{1},1)
   [val, idx] = min(abs(camera_timestamps{1}(i,1) - laser_timestamps(:,1)));
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

