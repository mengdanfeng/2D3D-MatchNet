clear all
clc

%root_dir = '/media/mengdan/data3/robotcar/grasshopper/sift_patch/';
new_root_dir = '/media/mengdan/data3/robotcar/grasshopper/sift_patch1/';

root_dir = '/media/mengdan/data3/robotcar/grasshopper/SIFT_And_ISS_v1/sift_patch_v1/';

folders = dir(root_dir);
folders = folders(3:end);

for i=1:length(folders)
    fprintf('progress: %d/%d\n', i, length(folders));
    folder_path = strcat(root_dir, folders(i).name);
    dest_path = strcat(new_root_dir, folders(i).name);
    if ~exist(dest_path, 'dir')
        mkdir(dest_path);
    end
    removeWhiteImage(folder_path, dest_path);
end

function removeWhiteImage(folder_path, dest_path)
% this function remove images with small variance
submap_folders = dir(folder_path);
submap_folders = submap_folders(3:end);

for i=1:length(submap_folders)
    fprintf('  submap progress: %d/%d\n', i, length(submap_folders));
    submap_folder = sprintf('%s/%s', folder_path, submap_folders(i).name);
    dest_submap_folder = sprintf('%s/%s', dest_path, submap_folders(i).name);
    if ~exist(dest_submap_folder, 'dir')
        mkdir(dest_submap_folder);
    end
    img_files = dir([submap_folder '/' '*.png']);
    for j=1:length(img_files)
        im = imread(sprintf('%s/%s', submap_folder, img_files(j).name));
        img = double(rgb2gray(im));        
        if var(img(:)) >= 260
            im_dest_path = sprintf('%s/%s', dest_submap_folder, img_files(j).name);
            imwrite(im, im_dest_path);
        end
    end
    
end   
    
end
% %
% root_dir = '/media/mengdan/data3/robotcar/grasshopper/sift_patch/2015-02-24-12-32-19/034/';
% %img_path = '2014-11-18-13-20-12/107/cam2_0015_0001_00001.png'; % 7.7729
% img_files = dir([root_dir '*.png']);
% img_variance = zeros(length(img_files),1);
% for i=1:length(img_files)
%     im = imread(strcat(root_dir,img_files(i).name));
%     img = double(rgb2gray(im));
%     img_variance(i)=var(img(:));
% end
% 
% idx = find(img_variance<=500);
% files = img_files(idx);
% for i=1:length(idx)
%     fprintf('%s %03f\n',files(i).name, img_variance(idx(i)));
% end

