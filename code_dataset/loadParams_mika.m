%%%%%%%%%%%%Folder Locations%%%%%%%%
%lidar
base_path='/media/deep-three/deep_ssd2/Robotcar/2015-11-13-10-28-08';
laser='lms_front';
laser_dir= strcat(base_path,'/',laser,'/');

%camera
cam_files_location='/media/deep-three/Deep_Store/RobotCar_Images/extracted_data/';
folder=base_path(find(base_path=='/',1,'last'):end);
camera_models_path='/media/deep-three/Deep_Store/RobotCar_Images/camera_models';
camera_base_path=strcat(cam_files_location,folder,'/');

camera{1}='mono_left';
camera{2}='mono_right';
camera_dir{1}=strcat(camera{1},'/');
camera_dir{2}=strcat(camera{2},'/');
% if strcmp(camera, 'stereo')
%     camera_dir='stereo\centre\';
% else
%     camera_dir=strcat(camera,'\');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%Load extrinsics%%%%%%%%
extrinsics_dir='/media/deep-three/Deep_Store/RobotCar_Images/extrinsics/';
laser_extrinisics = dlmread([extrinsics_dir 'lms_front.txt']);
ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);
camera_extrinsics{1} = dlmread(strcat(extrinsics_dir,camera{1},'.txt'));
camera_extrinsics{2} = dlmread(strcat(extrinsics_dir,camera{2},'.txt'));

G_ins_laser = SE3MatrixFromComponents(ins_extrinsics) \ SE3MatrixFromComponents(laser_extrinisics);
G_camera_ins{1} = SE3MatrixFromComponents(camera_extrinsics{1}) * SE3MatrixFromComponents(ins_extrinsics);
G_camera_ins{2} = SE3MatrixFromComponents(camera_extrinsics{2}) * SE3MatrixFromComponents(ins_extrinsics);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%Timestamps%%%%%%%%%%%%%
laser_timestamps = dlmread(strcat(base_path,'/',laser,'.timestamps'));
camera_timestamps{1}=dlmread(strcat(camera_base_path,camera{1},'.timestamps'));
camera_timestamps{2}=dlmread(strcat(camera_base_path,camera{2},'.timestamps'));
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
curvatureThresh = 0.8;
corrThreshold = 3;
track_length=3;
target_pc_size=4096;

%submap generation
submap_cover_distance=40.0;
camera_reading_distance=0.01;
camera_reading_angle=10;

% nFirstLaserRemove = 5000;
% nLastLaserRemove = 1000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%