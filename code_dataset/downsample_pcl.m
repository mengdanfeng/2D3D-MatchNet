close all
clear all
clc

% set path
pcd_file_root = '/media/mengdan/data3/robotcar/bumblebee/image_pcl_orig';
downsampled_pcd_root = '/media/mengdan/data3/robotcar/bumblebee/image_pcl';
folders = dir(pcd_file_root);
folders = folders(3:end);

for i=31:length(folders)
    folder = folders(i).name;
    files = dir([pcd_file_root '/' folder '/*.pcd']);
    
    % make directory
    dest_folder = sprintf('%s/%s', downsampled_pcd_root, folder);
    if ~exist(dest_folder, 'dir')
        mkdir(dest_folder);
    end

    for j=1:length(files)
        pcd_file = sprintf('%s/%s/%s', pcd_file_root, folder, files(j).name);
        pcl = pcread(pcd_file);
        
        % self-adaptive downsampling, downsample to roughly 6000-7000 points
        gridStep = 0.6;
        pcl_ds = pcdownsample(pcl, 'gridAverage', gridStep);       
        
        while true
            if pcl_ds.Count >= 6000 && pcl_ds.Count <= 7000
                break;
            elseif pcl_ds.Count > 7000                              
                gridStep = gridStep * 1.02;
                pcl_ds = pcdownsample(pcl, 'gridAverage', gridStep);
%                 fprintf('%s, %d, %s\n', num2str(j), pcl.Count, num2str(pcl_ds.Count));
            else
                gridStep = gridStep * 0.98;
                pcl_ds = pcdownsample(pcl, 'gridAverage', gridStep);
%                 fprintf('%s, %d, %s\n', num2str(j), pcl.Count, num2str(pcl_ds.Count));
            end
        end
        
        fprintf('Folder %d, file %d, orig: %d, downsample: %d\n', i, j, pcl.Count, pcl_ds.Count);
        
        % save pcl
        file_name = sprintf('%s/%s/%s', downsampled_pcd_root, folder, files(j).name);
        pcwrite(pcl_ds,file_name,'Encoding','ascii');  
    end
end
