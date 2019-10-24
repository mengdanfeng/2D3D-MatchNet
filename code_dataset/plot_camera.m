function plot_camera(G_ins_laser, laser_global_poses, ...
    submap_laser_idx, laser_timestamps, laser_dir, ...
    cam_global_poses, camera_submap_idx, G_camera_ins, G_camera_image,...
    camera_dir, camera_timestamps, LUT)

%submapIDs = [24,25];
pointcloud = [];
point_laser_idx = [];
cam_poses = {};

% use left_camera
% cam_side = 1;
% camera_timestamps = camera_timestamps{cam_side};
% camera_dir = camera_dir{cam_side};
% LUT = LUT{cam_side};

submapID = 24;
camera_idx = find(camera_submap_idx==submapID);
inv_initial_laser_global_pose = laser_global_poses{submap_laser_idx{submapID}(1)};

for k=1:length(camera_idx)
    camera_id = camera_idx(k);
    %cam_poses{end+1} = inv_initial_laser_global_pose\cam_global_poses{1}{camera_id} / G_camera_ins{1} * G_camera_image{1};
    cam_poses{end+1} = cam_global_poses{1}{camera_id} / G_camera_ins{1} * G_camera_image{1};
end


%for i=1:size(submap_laser_idx{submapID},2)  
for i=50:350
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

% pcshow(pointcloud', pointcloud(2,:));
% hold on

% plot predicted camera
file_dir = '/media/mengdan/data3/robotcar/grasshopper/results/2014-06-26-09-53-12';
pred_poses={};

% read pose file
file = sprintf('%s/%03d.txt', file_dir, submapID);
fid = fopen(file, 'r');
submap_poses = {};
line_id = 0;

img_line_id = 47;

while ~feof(fid)
    % the first 4 lines are not poses
    % read from the 5th line
    line = fgetl(fid);
    line_id = line_id + 1;        
    if line_id <= 4
        continue;
    end  
    
    % for specified line, read image, extract sift_uv and iss_xyz
    if line_id == img_line_id
        data = strsplit(line, ', ');
        
        % load image
        img_id = str2num(data{2});
        I = LoadImage(camera_dir{1}, camera_timestamps{1}(camera_idx(img_id),1), LUT{1});
        %figure(2), imshow(I);
        
        % read camera pose
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
        img_pose = inv_initial_laser_global_pose * rel_pose / G_camera_ins{1} * G_camera_image{1};     
        %img_pose = rel_pose / G_camera_ins{1} * G_camera_image{1};     
        
        % read sift_uv and iss_xyz
        inlier_num_id = 17;
        inlier_num = round(str2num(data{inlier_num_id}));
        
        sift_uv = zeros(1, 2*inlier_num);
        iss_xyz = zeros(1, 3*inlier_num);
        for m=1:inlier_num *2
            sift_uv(m) = str2num(data{inlier_num_id+m});
        end
        
        for n=1:inlier_num*3
            iss_xyz(n) = str2num(data{inlier_num_id+2*inlier_num+n});
        end
        
        sift_uv = reshape(sift_uv, 2, [])';
        iss_xyz = reshape(iss_xyz, 3, [])';
        
        % convert iss_xyz to global 
        homo_iss_xyz = [iss_xyz'; ones(1, size(iss_xyz',2))];
        global_iss_xyz_homo = inv_initial_laser_global_pose * homo_iss_xyz;
        iss_xyz = global_iss_xyz_homo(1:3,:);
    end          
    
%     if line_id > img_line_id 
%         break;
%     end

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
    %submap_poses{end+1} = rel_pose / G_camera_ins{1} * G_camera_image{1};
    submap_poses{end+1} = inv_initial_laser_global_pose * rel_pose / G_camera_ins{1} * G_camera_image{1};
end
fclose(fid);

% color = pointcloud(2,:)/max(pointcloud(2,:));
% pcshow(pointcloud',color);
% hold on

scatter3(iss_xyz(1,:), iss_xyz(2,:), iss_xyz(3,:), 60, [1 0 0],'LineWidth',2);
hold on

% plot single pred camera
location = img_pose(1:3,4);
orientation = img_pose(1:3,1:3);
cam = plotCamera('Location',location,'Orientation',orientation,'Size',0.5,'Opacity',0, 'Color', [1 0 0]);
%hold off

% connect camera center and iss
for i=1:size(iss_xyz, 2)
    x = [location(1), iss_xyz(1,i)];
    y = [location(2), iss_xyz(2,i)];
    z = [location(3), iss_xyz(3,i)];
    plot3(x,y,z, 'm','LineWidth',2);
    hold on
end

% plot gt camera
location = cam_poses{img_line_id-4}(1:3,4);
orientation = cam_poses{img_line_id-4}(1:3,1:3);
cam = plotCamera('Location',location,'Orientation',orientation,'Size',0.5,'Opacity',0, 'Color', [1 1 0]);

% insert image
xImage = [5735371 5735363; 5735371 5735363];
%xImage = [5735323 5735317; 5735323 5735317];    % The x data for the image corners
%yImage = [620676 620676; 620676 620676];             % The y data for the image corners
yImage = [620674 620674; 620674 620674];
zImage = [-114 -114;-107.3,-107.3];  % The z data for the image corners


% colormap('default')
axis off
grid off

s = surf(xImage,yImage,zImage,...    % Plot the surface
     'CData',I,...
     'FaceColor','texturemap');
s.EdgeColor = 'none';
hold on

% remove right side
%disp(size(pointcloud))
pointcloud(:,pointcloud(2,:)<620668) = [];
colormap('default')
cmap = colormap;
colors = (pointcloud(2,:)-min(pointcloud(2,:)))/ (max(pointcloud(2,:))-min(pointcloud(2,:))) * size(cmap, 1);
point_colors = zeros(size(pointcloud,2),3);
for i=1:size(cmap,1)
    idx = find(round(colors)==i);
    point_colors(idx,:) = repmat(cmap(i,:), length(idx), 1);
end
pcshow(pointcloud', point_colors);

set(gcf,'color','w');

% display image
figure,imshow(I);
 
    
    