root_dir = '/media/mengdan/data3/robotcar/bumblebee/';
image_root_dir = strcat(root_dir,'rgb_image');
pcl_root_dir = strcat(root_dir,'image_pcl');
image_folders = dir(image_root_dir);
image_folders = image_folders(3:end);
pcl_folders = dir(pcl_root_dir);
pcl_folders = pcl_folders(3:end);


for i=1:length(image_folders)
%     if(i==3 || i==11)
%         fprintf(' skip %d: %s\n',i,image_folders(i).name);
%         continue;
%     end    
    
    txt_file = sprintf('%strainval_txt/%s.txt', root_dir, image_folders(i).name);
    fprintf('%s\n',txt_file);
    fid = fopen(txt_file,'w');
    
    image_files = dir(sprintf('%s/%s', image_root_dir, image_folders(i).name));
    image_files = image_files(3:end);
    
    for j=1:length(image_files)
        image_file = sprintf('%s/%s/%s', image_root_dir, image_folders(i).name, image_files(j).name);
        [~, name, ~] = fileparts(image_files(j).name);
        pcl_file = sprintf('%s/%s/%s.pcd', pcl_root_dir, pcl_folders(i).name, name);
        
        % image file and pcl file
        line = sprintf('%s %s\n', image_file, pcl_file);
        
        % write to file
        fprintf(fid, line);
    end
    fclose(fid);
end
        
        
    
