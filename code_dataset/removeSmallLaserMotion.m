function [laser_timestamps,small_movement_id] =  removeSmallLaserMotion(laser_timestamps, vo_file, submap_params)

% submap_idx = [];
% submap_idx(1) = 1;
% sum_dist = 0;
% submap_counter = 1;
% cam_relative_poses = {};
% cam_relative_poses{1} = [];
% cam_relative_poses_Norm = [];
% cam_relative_poses_Norm(1,:) = zeros(1,2);
% relative_poses_counter = 1;
small_movement_id = [];

origin_id = 1;
for i=2:size(laser_timestamps,1)
    if(mod(i,100)==1)
        fprintf('progress: %d/%d\n', i, size(laser_timestamps,1));
    end
    % 4x4 relative poses    
    %fprintf('origin_id: %d, current_id: %d\n', origin_id, i);
    origin_timestamp = laser_timestamps(origin_id,1);
    current_timestamp = laser_timestamps(i,1);
    laser_rel_pose = RelativeToAbsolutePoses(vo_file,current_timestamp,origin_timestamp); 
    laser_rel_pose = laser_rel_pose{1};
    
    % 3x3 rotation matrix to 1x3 euler angle vector
    [alpha, belta, gamma] = getEulerAngles(laser_rel_pose(1:3,1:3));
    
    % relative distance for both angle and position
    laser_rel_pose_Norm = [norm([alpha, belta, gamma]), norm(laser_rel_pose(1:3,4))];
    %fprintf("pos: %03f, angle: %03f, \n",cam_rel_pose_Norm(1,1), cam_rel_pose_Norm(1,2));
    
    % remove laser poses with small movement
    if (laser_rel_pose_Norm(1,1) < submap_params.laser_reading_angle && laser_rel_pose_Norm(1,2) < submap_params.laser_reading_distance)
        small_movement_id = [small_movement_id; i];
        %fprintf("small movement, camera: %d, pos: %03f, angle: %03f, \n", i,cam_rel_pose_Norm(1,1), cam_rel_pose_Norm(1,2));
        continue;        
    end    
    
    origin_id = i;
    
%     % record relative poses and pose norm
%     relative_poses_counter = relative_poses_counter + 1;
%     %fprintf('relative_pose_counter: %d\n', relative_poses_counter);
%     cam_relative_poses{relative_poses_counter} = laser_rel_pose;
%     cam_relative_poses_Norm(relative_poses_counter,:) = laser_rel_pose_Norm;
    
%     % travelled distance
%     sum_dist = sum_dist + cam_relative_poses_Norm(relative_poses_counter,2);
%     
%     if sum_dist <= submap_params.submap_cover_distance  
%         submap_idx(relative_poses_counter) = submap_counter;
%         continue;
%     else
%         sum_dist = 0;
%         submap_counter = submap_counter + 1;
%         submap_idx(relative_poses_counter) = submap_counter;
%     end        

end

% remove camera timestamps with small movement
laser_timestamps(small_movement_id,:) = [];

end