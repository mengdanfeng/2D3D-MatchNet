data_root = '/media/mengdan/data3/robotcar/grasshopper';
submap_root = strcat(data_root, '/lms_front_submap/');

date_time = '2014-06-26-09-53-12';
submap_folder = strcat(submap_root, date_time);

submap_id = 25;
submap_file = sprintf('%s/%03d.pcd', submap_folder, submap_id);

submap_pcl = pcread(submap_file);
pcshow(submap_pcl);

