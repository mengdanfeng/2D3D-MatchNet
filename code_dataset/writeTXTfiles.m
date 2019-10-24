root_dir = '/media/mengdan/data3/robotcar/grasshopper/';
sift_root_dir = strcat(root_dir,'sift_patch');
iss_root_dir = strcat(root_dir,'iss_volume');
sift_folders = dir(sift_root_dir);
sift_folders = sift_folders(3:end);
iss_folders = dir(iss_root_dir);
iss_folders = iss_folders(3:end);


for i=1:length(sift_folders)
    if(i==3 || i==11)
        fprintf(' skip %d: %s\n',i,sift_folders(i).name);
        continue;
    end    
    
    txt_file = sprintf('%stxt_files/%s.txt', root_dir, sift_folders(i).name);
    fprintf('%s\n',txt_file);
    fid = fopen(txt_file,'w');
    sift_submap_folders = dir(sprintf('%s/%s', sift_root_dir, sift_folders(i).name));
    sift_submap_folders = sift_submap_folders(3:end);
%     iss_submap_folders = dir(sprintf('%s/%s', iss_root_dir, iss_folders(i).name));
%     iss_submap_folders = iss_submap_folders(3:end);
    for j=1:length(sift_submap_folders)
        sift_submap_files = dir(sprintf('%s/%s/%s', sift_root_dir, sift_folders(i).name, sift_submap_folders(j).name));
        sift_submap_files = sift_submap_files(3:end);
%         iss_submap_files = dir(sprintf('%s/%s/%s', iss_root_dir, iss_folders(i).name, iss_submap_folders(j).name));
%         iss_submap_files = iss_submap_files(3:end);
        for k=1:length(sift_submap_files)
            sift_file_name = sprintf('%s/%s/%s/%s', sift_root_dir, sift_folders(i).name, sift_submap_folders(j).name, sift_submap_files(k).name);
            [~,name,~]=fileparts(sift_submap_files(k).name);
           
            iss_file_name = sprintf('%s/%s/%s/%s.pcd', iss_root_dir,sift_folders(i).name, sift_submap_folders(j).name,name);
            
            %line = sprintf('%s %s %s %s\n', sift_submap_folders(j).name, name(4), sift_file_name, iss_file_name);
            line = sprintf('%s %s %s %s\n', sift_submap_folders(j).name, name(4), sift_file_name, iss_file_name);
            
            fprintf(fid, line);            
        end
    end
    fclose(fid);
end
        
    
