function build_pointclouds(G_ins_laser, laser_global_poses, ...
    submap_laser_idx, laser_timestamps, laser_dir, ...
    cam_global_poses, camera_submap_idx, G_camera_ins, G_camera_image)

submapIDs = [24,25];
pointcloud = [];
point_laser_idx = [];
cam_poses = {};
%inv_initial_laser_global_pose = laser_global_poses{submap_laser_idx{submapIDs(1)}(1)};
for j=1:length(submapIDs)
    submapID = submapIDs(j);
    camera_ids = find(camera_submap_idx==submapID);
    for k=1:length(camera_ids)
        camera_id = camera_ids(k);
        cam_poses{end+1} = cam_global_poses{1}{camera_id} / G_camera_ins{1} * G_camera_image{1};
        %cam_poses{end+1} = cam_global_poses{1}{camera_id} / G_camera_ins{1};
    end
    
    for i=1:size(submap_laser_idx{submapID},2)    
    %for i=1:500
        laser_id = submap_laser_idx{submapID}(i);
        scan_path = [laser_dir '/' num2str(laser_timestamps(laser_id,1)) '.bin'];
        scan_file = fopen(scan_path);
        scan = fread(scan_file, 'double');
        fclose(scan_file);

        scan = reshape(scan, [3 numel(scan)/3]);
        scan(3,:) = zeros(1, size(scan,2));

        
        %scan = inv_initial_laser_global_pose \ (laser_global_poses{laser_id} * G_ins_laser) * [scan; ones(1, size(scan,2))];
        scan = (laser_global_poses{laser_id} * G_ins_laser) * [scan; ones(1, size(scan,2))];
        pointcloud = [pointcloud scan(1:3,:)];
        point_laser_idx = [point_laser_idx laser_id*ones(1,size(scan,2))];

    end
end 

pcshow(pointcloud', pointcloud(2,:));

hold on

% plot ground truth camera
for i=1:length(cam_poses)
    if mod(i, 4) == 1
        location = cam_poses{i}(1:3,4);
        orientation = cam_poses{i}(1:3,1:3);
        cam = plotCamera('Location',location,'Orientation',orientation,'Size',0.4,'Opacity',0, 'Color', [1 0.9 0.1]);
        hold on
    end
end

% plot predicted camera
file_dir = '/media/mengdan/data3/robotcar/grasshopper/results/2014-06-26-09-53-12';
pred_poses={};
for i=1:length(submapIDs)
    file = sprintf('%s/%03d.txt', file_dir, submapIDs(i));
    fid = fopen(file, 'r');
    submap_poses = {};
    initial_laser_global_pose = laser_global_poses{submap_laser_idx{submapIDs(i)}(1)};
    line_id = 0;
    while ~feof(fid)
        % the first 4 lines are not poses
        % read from the 5th line
        line = fgetl(fid);
        line_id = line_id + 1;        
        if line_id <= 4
            continue;
        end  
        
        data = strsplit(line, ', ');

        % pose is None, skip
        if strcmp(data{3}, 'None')
            continue;
        end
        % large pose error, skip
        if (str2num(data{3}) > 10 || str2num(data{4}) > 0.8)
            continue;
        end
        
        % skip cam_id = 2
        if str2num(data{1}) == 2
            break;
        end
        
        %fprintf('  line_id: %d, data{1}: %s, data{2}: %s, data{3}: %s\n', line_id,data{1}, data{2}, data{3});
        
        % keep suitable pose
        tvec = zeros(3,1);
        rmat = zeros(9,1);
        for ii = 1:3
            tvec(ii,:) = str2num(data{ii+4});
        end
        for jj=1:9
            rmat(jj,:) = str2num(data{jj+7});
        end
        %disp(rmat)
        rmat = reshape(rmat, [3,3])';
        %disp(rmat)
            
        % get relative poses
        rel_pose = [rmat, tvec; 0,0,0,1];
        
        % get global poses
        submap_poses{end+1} = initial_laser_global_pose * rel_pose / G_camera_ins{1} * G_camera_image{1};
        % submap_poses{end+1} = initial_laser_global_pose * rel_pose;
    end
    fclose(fid);
    pred_poses{i} = submap_poses;
end

% plot predicted camera poses
for k=1:length(pred_poses)
    cam_poses = pred_poses{k};
    for i=1:length(cam_poses)
        location = cam_poses{i}(1:3,4);
        orientation = cam_poses{i}(1:3,1:3);
        cam = plotCamera('Location',location,'Orientation',orientation,'Size',0.5,'Opacity',0, 'Color', [1 0 0]);
        hold on
    end
end

grid off
axis off

    
    