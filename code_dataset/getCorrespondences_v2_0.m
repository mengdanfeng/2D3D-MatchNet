% search for correspondences
%  list of img uv correspondences to the points --> point_to_uv_correspondences{pointID} = [imageID uv; ...]

% sift_to_iss_corespondences{1=left/2=right}{imageID} = [sift_id, sift_uv, iss_id, iss_uv]

function sift_to_iss_corespondences = ...
    getCorrespondences_v2(images, pointcloud, cam_idx, image_features, ...
    reproj_point_img_uv_list, reproj_point_img_idx_list, corrThreshold,...
    valid_keypoints_index, submapID, cam_id, submap_sift_iss_gt_corr_dir)

sift_to_iss_corespondences = cell(length(cam_idx), 1);
for imgID=1:length(cam_idx)
    
    % update 
    orig_iss_id = 1:length(valid_keypoints_index);      % number of valid iss keypoints
    orig_sift_id = 1:size(image_features{imgID},2);     % number of valid sift keypoints
    [reproj_point_img_idx_list{imgID},ia,ib] = intersect(reproj_point_img_idx_list{imgID},valid_keypoints_index);
    reproj_point_img_uv_list{imgID} = reproj_point_img_uv_list{imgID}(ia,:);
    
    % find sift features to 3d reprojected point correspondences
%    [idx, dist] = knnsearch(image_features{imgID}(1:2,:)', reproj_point_img_uv_list{imgID},'dist','euclidean','k',1);
%    correspondences = [idx (1:size(reproj_point_img_uv_list{imgID},1))'];

    [idx, dist] = knnsearch(reproj_point_img_uv_list{imgID},image_features{imgID}(1:2,:)', 'dist','euclidean','k',1);
%     correspondences = [idx (1:size(reproj_point_img_uv_list{imgID},1))'];
    
    % select correspondences < 3 pixels
    dist_idx = find(dist < corrThreshold);
%     correspondences = correspondences( dist_idx,:);
    
    % get image features
%     feats = image_features{imgID}(1:2,correspondences(:,1));
    
%     %%% add feature scale
%     feats_scale = image_features{imgID}(3,correspondences(:,1))./1.6;
    
    % get reprojected 3d points
%     reprojPts = reproj_point_img_uv_list{imgID}(correspondences(:,2),:);
    file_path = sprintf('%s/cam%d_%03d', submap_sift_iss_gt_corr_dir, cam_id, imgID);
    fid = fopen(file_path, 'w');
    for i = 1 : length(dist_idx)
        sift_id = dist_idx(i);
        sift_uv = image_features{imgID}(1:2, sift_id)';
        iss_id = idx(dist_idx(i));
        iss_uv = reproj_point_img_uv_list{imgID}(iss_id);
        sift_to_iss_corespondences{imgID} = [sift_to_iss_corespondences{imgID};sift_id, sift_uv, iss_id, iss_uv];
        
        % write gt correspondences to file
        fprintf(fid, '%d %d\n', sift_id, iss_id);
    end
    fclose(fid);  
    
    
end
