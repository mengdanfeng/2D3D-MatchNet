function [camera_timestamps, camera_laser_idx, cam_global_poses] = ...
    removeCamPoseWOINS_2(camera_timestamps, camera_laser_idx, cam_global_poses, laser_global_poses)

cnt=1;
idx=zeros(size(camera_timestamps,1),1);
for i=2:size(camera_timestamps,1)
   
    % check that laser pose is valid (including the past 5 laser poses)
    idx1=camera_laser_idx(i)-5;
    if idx1 <= 0, idx1=1; end
    idx2=camera_laser_idx(i);
   
    valid=1;
    for j=idx1:idx2
        %disp(j)
        if isnan(laser_global_poses{j}(1,4)) || isnan(laser_global_poses{j}(2,4)) || isnan(laser_global_poses{j}(3,4))
            valid=0;
            j=idx2;
            %fprintf('  nan, i,j, idx1, idx2, valid: %d,%d,%d, %d, %d, %d\n',i, j, idx1, idx2, valid);
        end
        %fprintf('  i,j, idx1, idx2, valid: %d,%d,%d, %d, %d, %d\n',i, j, idx1, idx2, valid);
    end

    % add valid INS poses (NAN is non-valid)
    if ~(isnan(cam_global_poses{i}(1,4)) || isnan(cam_global_poses{i}(2,4)) || isnan(cam_global_poses{i}(3,4))) && valid
       idx(cnt) = i; 
       cnt = cnt + 1;
    end
end
idx = idx(1:cnt-1);
camera_timestamps = camera_timestamps(idx,:);
camera_laser_idx = camera_laser_idx(idx,:);
cam_global_poses = cam_global_poses(idx,1);
