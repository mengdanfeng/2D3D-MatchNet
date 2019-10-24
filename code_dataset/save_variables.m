camera_info.camera_timestamps = camera_timestamps;
camera_info.camera_submap_idx = camera_submap_idx;
camera_info.num_submap = num_submap;

laser_info.laser_timestamps = laser_timestamps;
laser_info.submap_laser_idx = submap_laser_idx;
laser_info.submap_pointclouds = submap_pointclouds;
laser_info.submap_keypoints = submap_keypoints;

save('camera_info.mat', 'camera_info');
save('laser_info.mat', 'laser_info');
