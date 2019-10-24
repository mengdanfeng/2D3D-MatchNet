function [landmark_frame_ids, landmark_timestamps, landmark_laser_idx, landmark_global_poses] = ...
    extractLandmarkImages(camera_timestamps, camera_laser_idx, cam_global_poses, landmark_dist)
% this function is to extract images at 15m interval 

landmark_frame_ids = [1];

num_images = length(cam_global_poses);
travel_dist = 0.0;
for i=2:num_images
    start_pose = cam_global_poses(i-1);
    next_pose = cam_global_poses(i);
    pose_dist = norm(next_pose(1:2,4) - start_pose(1:2,4));
    travel_dist = travel_dist + pose_dist;
    
    % check travel dist
    if(travel_dist >= landmark_dist)
        landmark_frame_ids = [landmark_frame_ids; i];
        travel_dist = 0.0;
    end
        
end

landmark_frame_ids = landmark_frame_ids(1:end-1);
landmark_timestamps = camera_timestamps(landmark_frame_ids,:);
landmark_global_poses = cam_global_poses(landmark_frame_ids);
landmark_laser_idx = camera_laser_idx(landmark_frame_ids);

end

% function [landmark_image, landmark_pcl, is_valid] = ...
%     extractLandmarkPCL(landmark_frame_ids, landmark_laser_idx)
% % this function is to build pcl for each landmark image
% 
% for i=1:length()
% 
% 
% 
% end

