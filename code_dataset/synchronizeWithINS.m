function [pose_timestamps] = synchronizeWithINS(ins_file, pose_timestamps)
  
% remove camera_timestamps without ins

  ins_file_id = fopen(ins_file);
  headers = textscan(ins_file_id, '%s', 15, 'Delimiter',',');
  ins_data = textscan(ins_file_id, ...
      '%u64 %s %f %f %f %f %f %f %s %f %f %f %f %f %f','Delimiter',',');
  fclose(ins_file_id);
  
  
  ins_timestamps = ins_data{1};
  pose_timestamps_col1 = pose_timestamps(:,1);
  
%   lower_index = max(find(ins_timestamps(:,1)<=min(pose_timestamps), 1, ...
%       'last')-1,1);
  if(min(ins_timestamps(:,1))<=min(pose_timestamps_col1))
      %lower_index = find(ins_timestamps(:,1)<=min(pose_timestamps_col1), 1, 'last')-1;
      camera_lower_idx = 1;
  else
      lower_index = 1;
      camera_lower_idx = find(pose_timestamps_col1 > ins_timestamps(lower_index, 1), 1, 'first');
  end
  
  if(max(ins_timestamps(:,1))>=max(pose_timestamps_col1))
      %upper_index = find(ins_timestamps(:,1)>=max(pose_timestamps_col1), 1, 'first')+1;
      camera_upper_idx = length(pose_timestamps_col1);
  else
      upper_index = length(ins_timestamps(:,1));
      camera_upper_idx = find(pose_timestamps_col1 < ins_timestamps(upper_index, 1), 1, 'last');     
  end
  
  fprintf('lower_id: %d, upper_id: %d, expected: (1,%d)\n', camera_lower_idx, camera_upper_idx, size(pose_timestamps,1));
  pose_timestamps = pose_timestamps(camera_lower_idx:camera_upper_idx,:);