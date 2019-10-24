iss_xyz_root = '/media/mengdan/data3/robotcar/grasshopper/iss_volume/2014-06-26-09-53-12';
sift_uv_root = '/media/mengdan/data3/robotcar/grasshopper/sift_patch/2014-06-26-09-53-12';

iss_xyz_txt_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_index_value/2014-06-26-09-53-12';
sift_uv_txt_root = '/media/mengdan/data3/robotcar/grasshopper/2d_3d_index_value/2014-06-26-09-53-12';
for submap_id=1:48
    folder = sprintf('%s/%03d', iss_xyz_txt_root, submap_id);
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
end

max_submap_id = 47;

for submap_id = 1:max_submap_id    
    fprintf( '-- convert iss: %d/%d\n', submap_id, max_submap_id);
    convertISSMatToTxt(iss_xyz_root, submap_id, iss_xyz_txt_root);
end

for submap_id = 1:max_submap_id
    fprintf('-- convert sift to iss: %d/%d\n', submap_id, max_submap_id);
    folders = dir(sprintf('%s/%03d', sift_uv_root, submap_id));
    folders = folders(3:end);
    for j=1:length(folders)
        folder = folders(j).name;
        cam_id = str2num(folder(4));
        img_id = str2num(folder(6:8));
        convertSIFTMatToTxt(sift_uv_root, submap_id, cam_id, img_id, sift_uv_txt_root);
    end
end   


function convertISSMatToTxt(iss_xyz_root, submap_id, iss_xyz_txt_root)
% this function convert .mat file to .xyz 
iss_mat_filename = sprintf('%s/%03d/%03d.mat', iss_xyz_root, submap_id, submap_id);
load(iss_mat_filename)      % cell, 1xN(valid_iss_points), get iss_xyz

valid_point_num = length(iss_xyz);
valid_iss_xyz = zeros(valid_point_num,4);
for i=1:valid_point_num     
    valid_iss_xyz(i,:) = [i, iss_xyz{i}{3}];
end

% write iss_xyz to txt file
iss_txt_filename = sprintf('%s/%03d/iss_%03d.txt', iss_xyz_txt_root, submap_id, submap_id);
dlmwrite(iss_txt_filename, valid_iss_xyz,  'precision', '%.6f');
end

function convertSIFTMatToTxt(sift_uv_root, submap_id, cam_id, img_id, sift_uv_txt_root)
% this function convert .mat file to .xyz 
sift_mat_filename = sprintf('%s/%03d/cam%d_%03d/%03d_%03d.mat', sift_uv_root, submap_id, cam_id,...
    img_id, submap_id, img_id);
load(sift_mat_filename)      % cell, 1xN(valid_sift_points), get sift_pos

valid_point_num = length(sift_pos);
valid_sift_uv = zeros(valid_point_num,3);
for i=1:valid_point_num     
    valid_sift_uv(i,:) = [i, sift_pos{i}{3}'];
end

% write sift_uv to txt file
sift_txt_filename = sprintf('%s/%03d/cam%d_%03d.txt', sift_uv_txt_root, submap_id, cam_id, img_id);
dlmwrite(sift_txt_filename, valid_sift_uv,  'precision', '%.6f');

end