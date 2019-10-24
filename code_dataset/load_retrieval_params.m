%% Folder root location 
base_path = '/media/mengdan/data3/robotcar/grasshopper';
camera_base_path = '/media/mengdan/DATA1/mengdan/datasets/RobotCar_Stereo/extracted_data';
laser_base_path = '/media/mengdan/data3/robotcar/grasshopper/lms_front';
data_base_path = '/media/mengdan/data3/robotcar/bumblebee';
gps_base_path = '/media/mengdan/DATA1/mengdan/datasets/RobotCar_Stereo/gps';
camera_models_path = strcat(base_path, '/', 'robotcar-dataset-sdk/models/');

%% generated image and pcl folder location
rgb_image_path = '/media/mengdan/data3/robotcar/bumblebee/rgb_image';
image_pcl_path = '/media/mengdan/data3/robotcar/bumblebee/image_pcl';

%% List all runs
all_date_times = dir(camera_base_path);
all_date_times = all_date_times(3:end);
 
for i=1:length(all_date_times)
    date_time = all_date_times{i}.name;
    date_time_image_folder = sprintf('%s/%s', rgb_image_path, date_time);
    date_time_pcl_folder = sprintf('%s/%s', image_pcl_path, date_time);
    if(~exist(date_time_image_folder, 'dir'))
        mkdir(date_time_image_folder);
    end
    
    if(~exist(date_time_pcl_folder,'dir'))
        mkdir(date_time_pcl_folder);
    end
end        

%% Sensor type
camera_type = 'stereo/centre';
gps_type = 'gps/ins.csv';
laser_type = 'lms_front';

%% Load extrinsics
extrinsics_dir = strcat(base_path, '/', 'robotcar-dataset-sdk/extrinsics/');
laser_extrinisics = dlmread([extrinsics_dir 'lms_front.txt']);
ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);
camera_extrinsics = dlmread(strcat(extrinsics_dir,camera_type,'.txt'));

G_ins_laser = SE3MatrixFromComponents(ins_extrinsics) \ SE3MatrixFromComponents(laser_extrinisics);
G_camera_ins = SE3MatrixFromComponents(camera_extrinsics) * SE3MatrixFromComponents(ins_extrinsics);


%% Landmark/Submap image parameters
landmark_dist = 15;

