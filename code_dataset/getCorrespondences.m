% search for correspondences
%  list of img uv correspondences to the points --> point_to_uv_correspondences{pointID} = [imageID uv; ...]

function point_to_uv_correspondences = ...
    getCorrespondences(images, pointcloud, cam_idx, image_features, ...
    reproj_point_img_uv_list, reproj_point_img_idx_list, corrThreshold)

point_to_uv_correspondences = cell(size(pointcloud,2), 1);
for imgID=1:length(cam_idx)
    
    % find sift features to 3d reprojected point correspondences
    [idx, dist] = knnsearch(image_features{imgID}(1:2,:)', reproj_point_img_uv_list{imgID},'dist','euclidean','k',1);
    correspondences = [idx (1:size(reproj_point_img_uv_list{imgID},1))'];
    
    % select correspondences < 3 pixels
    correspondences = correspondences( dist < corrThreshold,:);
    
    % get image features
    feats = image_features{imgID}(1:2,correspondences(:,1));
    
    %%% add feature scale
    feats_scale = image_features{imgID}(3,correspondences(:,1))./1.6;
    
    % get reprojected 3d points
    reprojPts = reproj_point_img_uv_list{imgID}(correspondences(:,2),:);
        
    % get 3d point indices 
    point_idx = reproj_point_img_idx_list{imgID}(correspondences(:,2));
    for j=1:length(point_idx)
        pointID = point_idx(j);
        point_to_uv_correspondences{pointID} = [point_to_uv_correspondences{pointID}; imgID reprojPts(j,:) feats(:,j)' feats_scale(:,j)'];
    end
    
%     figure,
%     imshow(images{imgID})
%     hold on
%     plot(feats(1,:), feats(2,:), 'r.')
%     plot(reprojPts(:,1), reprojPts(:,2), 'b.')
%     hold off
%         
% pause
    
end