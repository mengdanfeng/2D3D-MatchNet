function pointcloud = build_pointcloud(G_ins_laser, laser_global_poses, ...
    submap_laser_idx, submapID, laser_timestamps, laser_dir)

pointcloud = [];
inv_initial_laser_global_pose = laser_global_poses{submap_laser_idx{submapID}(1)};
for i=1:size(submap_laser_idx{submapID},2)
    
    laser_id = submap_laser_idx{submapID}(i);
    scan_path = [laser_dir '/' num2str(laser_timestamps(laser_id,1)) '.bin'];
    scan_file = fopen(scan_path);
    scan = fread(scan_file, 'double');
    fclose(scan_file);

    scan = reshape(scan, [3 numel(scan)/3]);
    scan(3,:) = zeros(1, size(scan,2));

    scan = inv_initial_laser_global_pose \ (laser_global_poses{laser_id} * G_ins_laser) * [scan; ones(1, size(scan,2))];
    pointcloud = [pointcloud scan(1:3,:)];
end
