function pointcloud = BuildPCLVO(laser_timestamps, laser_dir, G_ins_laser, vo_file)
%% Function: build pointcloud from specified laser scan data and their poses from odom
% input:
%   -- laser_timestamps: all the laser timestamps of current submap, Nx1
%   -- laser_dir: laser scans directory
%   -- G_ins_laser: transformation from laser to ins, 4x4
%   -- vo_file: odom data 
% output:
%   -- pointcloud: reconstructed pointcloud with respect to the first laser
%   scan frame.

%% the poses of all laser scan relative to the origin laser scan.
pointcloud = [];
relative_poses = RelativeToAbsolutePoses(vo_file, reshape(laser_timestamps(1:end),1,[]), laser_timestamps(1));

for i=1:length(laser_timestamps)
    fprintf('%d/%d\n',i,length(laser_timestamps));
    scan_path = [laser_dir '/' num2str(laser_timestamps(i)) '.bin'];
    scan_file = fopen(scan_path);
    scan = fread(scan_file, 'double');
    fclose(scan_file);
    
    scan = reshape(scan, [3 numel(scan)/3]);
    scan(3,:) = zeros(1, size(scan,2));

    scan = relative_poses{i} * G_ins_laser * [scan; ones(1, size(scan,2))];
    pointcloud = [pointcloud scan(1:3,:)];
end

% visualize
%pcl = pointCloud(pointcloud');
%figure, pcshow(pcl);


end

    

    
        