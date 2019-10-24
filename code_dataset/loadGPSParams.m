%%%%%%%%%%%%Folder Locations%%%%%%%%
%pick one run
base_path = '/media/mengdan/data3/robotcar/grasshopper';
date_time = '2014-06-26-09-53-12';

%lidar
laser='lms_front';
laser_dir= strcat(base_path,'/',laser,'/', date_time, '/', laser);

% vo
vo = 'vo';
vo_file = strcat(base_path, '/', vo, '/', date_time, '/', vo, '/', 'vo.csv');

%lidar submaps dir
laser_submap_dir = 'lms_front_submap';

% pcd dir
pcd_dir = strcat(base_path, '/lms_front_submap/',date_time, '/');
keypoints_dir = strcat(base_path, '/lms_front_keypoints/', date_time, '/');
%submap_keypoints_dir = strcat(base_path, '/submap_keypoints/', date_time, '/');
pcl_proj_dir = strcat(base_path, '/lms_front_proj/', date_time, '/');

% write sift patch and iss voxel
sift_patch_dir = strcat(base_path, '/sift_patch/', date_time, '/');
iss_volume_dir = strcat(base_path, '/iss_volume/', date_time, '/');
sift_iss_proj_dir = strcat(base_path, '/sift_iss_proj/', date_time, '/');

if ~exist(pcd_dir,'dir')
    mkdir(pcd_dir);
end

if ~exist(keypoints_dir,'dir')
    mkdir(keypoints_dir);
end

% if ~exist(submap_keypoints_dir,'dir')
%     mkdir(submap_keypoints_dir);
% end

if ~exist(pcl_proj_dir,'dir')
    mkdir(pcl_proj_dir);
end

if ~exist(sift_patch_dir, 'dir')
    mkdir(sift_patch_dir);
end

if ~exist(iss_volume_dir, 'dir')
    mkdir(iss_volume_dir);
end

if ~exist(sift_iss_proj_dir, 'dir')
    mkdir(sift_iss_proj_dir);
end


%camera
camera{1}='mono_left';
camera{2}='mono_right';
camera_dir{1}=strcat(base_path, '/', camera{1},'/', date_time, '/', camera{1});
camera_dir{2}=strcat(base_path, '/', camera{2},'/', date_time, '/', camera{2});

camera_models_path = strcat(base_path, '/', 'robotcar-dataset-sdk/models/');
%cam_files_location='E:\RobotCar';
%folder=base_path(find(base_path=='\',1,'last'):end);
%camera_models_path='E:\RobotCar\models\';

%camera_base_path=strcat(cam_files_location,folder,'\');


% if strcmp(camera, 'stereo')
%     camera_dir='stereo\centre\';
% else
%     camera_dir=strcat(camera,'\');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%Load extrinsics%%%%%%%%
%extrinsics_dir='E:\RobotCar\extrinsics\';
extrinsics_dir = strcat(base_path, '/', 'robotcar-dataset-sdk/extrinsics/');
laser_extrinisics = dlmread([extrinsics_dir 'lms_front.txt']);
ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);
camera_extrinsics{1} = dlmread(strcat(extrinsics_dir,camera{1},'.txt'));
camera_extrinsics{2} = dlmread(strcat(extrinsics_dir,camera{2},'.txt'));

G_ins_laser = SE3MatrixFromComponents(ins_extrinsics) \ SE3MatrixFromComponents(laser_extrinisics);
G_camera_ins{1} = SE3MatrixFromComponents(camera_extrinsics{1}) * SE3MatrixFromComponents(ins_extrinsics);
G_camera_ins{2} = SE3MatrixFromComponents(camera_extrinsics{2}) * SE3MatrixFromComponents(ins_extrinsics);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%Timestamps%%%%%%%%%%%%%
laser_timestamps = dlmread(strcat(laser_dir,'.timestamps'));
camera_timestamps{1}=dlmread(strcat(camera_dir{1}, '.timestamps'));
camera_timestamps{2}=dlmread(strcat(camera_dir{2}, '.timestamps'));
camera_timestamps{1}((camera_timestamps{1}(:,2) == 0),:) = [];
camera_timestamps{2}((camera_timestamps{2}(:,2) == 0),:) = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%Camera Model%%%%%%%%%%%%%
LUT=cell(2,1);
G_camera_image=cell(2,1);
[ fx, fy, cx, cy, G_camera_image{1}, LUT{1}] = ReadCameraModel(camera_dir{1},camera_models_path);
camera_intrinsics{1}=[fx,0,cx;0,fy,cy;0,0,1];
[ fx, fy, cx, cy, G_camera_image{2}, LUT{2}] = ReadCameraModel(camera_dir{2},camera_models_path);
camera_intrinsics{2}=[fx,0,cx;0,fy,cy;0,0,1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%Parameters%%%%%%%%%%%%%%%
rangeSearchThresh = 0.3;
curvatureThresh = 0.5;
corrThreshold = 3;
track_length=3;
target_pc_size=4096;

% camera param
camera_reading_angle= 10;
camera_reading_distance=0.02;
submap_cover_distance=60.0;

% laser param
laser_reading_angle = 2;
laser_reading_distance = 0.005;

% grid resolution
grid_resolution.x = 0.1;
grid_resolution.y = 0.1;
grid_resolution.z = 0.1;

% patch information
% patch_width = 128;
% patch_height = 128;
% 
volume_radius = 1.0;
% min_point_in_volume = 200;

% nFirstLaserRemove = 5000;
% nLastLaserRemove = 1000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%