% Compute sift features for each image in the submap

function [images, image_features] = ...
    computeImageFeatures(cam_idx, camera_dir, camera_timestamps, LUT)

images=cell(length(cam_idx), 1);
image_features=cell(length(cam_idx), 1);
for imgID=1:length(cam_idx)
    
    fprintf('      compute sift: %d of %d images\n', imgID, length(cam_idx));
    I = LoadImage(camera_dir, camera_timestamps(cam_idx(imgID),1), LUT);
    if isempty(I)
        fprintf('  image does not exist. dir:%s, time: %f\n', camera_dir,camera_timestamps(cam_idx(imgID)));
    end
    I_gray= single(rgb2gray(I));
    f=vl_sift(I_gray);
    image_features{imgID}=f;
    images{imgID}=I;
    
%     figure,
%     imshow(I)
%     hold on
%     plot(f(1,:), f(2,:), 'g.')
%     hold off
%     pause
end