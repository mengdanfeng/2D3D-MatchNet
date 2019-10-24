function [pose_timestamps] = removePoseWOData(pose_timestamps, data_dir, data_source)
% remove camera/laser timestmaps without image/laser scan 

if data_source == 1     % 1 means camera, 2 means laser
    data_files = dir([data_dir '/' '*.png']);
elseif data_source == 2
    data_files = dir([data_dir '/' '*.bin']);
else
    error('Data source invalid, choose from camera or laser.\n');
end

[~,data_files_name,~] = cellfun(@fileparts,{data_files.name},'UniformOutput',false);
data_files_name = (cellfun(@str2double, data_files_name))';

%[idx1,idx2] = find(pose_timestamps(:,1) == data_files_name);
[~, idx1, idx2] = intersect(pose_timestamps(:,1),data_files_name);

if(length(idx1) ~= length(pose_timestamps))
    fprintf('  %s data from %s are not the same as pose_timestmaps\n', data_source, data_dir);
end

pose_timestamps = pose_timestamps(idx1,:);
    
end

