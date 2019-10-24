function [laser_dir, camera_dir, laser_timestamps, camera_timestamps, camera_intrinsics, G_camera_image, LUT] = ...
    getParamsFromDatetime(date_time)

% this function gets related params from the given date_time

%lidar dir
laser_dir= strcat(laser_base_path, '/', date_time, '/', laser_type);

%camera dir
camera_dir = strcat(camera_base_path, '/', date_time, '/', camera_type);

% Timestamps
laser_timestamps = dlmread(strcat(laser_dir,'.timestamps'));
camera_timestamps=dlmread(strcat(camera_dir, '.timestamps'));
camera_timestamps((camera_timestamps(:,2) == 0),:) = [];

% camera model
[ fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel(camera_dir,camera_models_path);
camera_intrinsics = [fx,0,cx;0,fy,cy;0,0,1];

end

