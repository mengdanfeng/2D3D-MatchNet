base_path='/media/deep-three/Deep_Store/RobotCar_Images/extracted_data/2014-05-19-13-05-38/';
camera_path='stereo/centre/';
timestamps=dlmread(strcat(base_path,'stereo.timestamps'));
ins_file=strcat(base_path,'gps/ins.csv');

camera_models_path='/media/deep-three/Deep_Store/RobotCar_Images/camera_models/';

%get LUT
[ ~, ~, ~, ~, ~, LUT] = ReadCameraModel(camera_path,camera_models_path);

no_tracked_frames=5;
no_todisplay=25;
for i=100:size(timestamps,1)

    Ia = LoadImage(strcat(base_path,camera_path), timestamps(i,1), LUT);
    Ia_gray= single(rgb2gray(Ia));
    [fa,da]=vl_sift(Ia_gray);
%     [fa,da]=vl_sift(Ia_gray,'PeakThresh', 2);
    
    for j=i+5:i+no_tracked_frames
        
        Ib = LoadImage(strcat(base_path,camera_path), timestamps(j,1), LUT);
        Ib_gray = single(rgb2gray(Ib));
        [fb,db]=vl_sift(Ib_gray);
%         [fb,db]=vl_sift(Ib_gray, 'PeakThresh',2);
        
        [matches, scores] = vl_ubcmatch(da,db) ;

        [drop, perm] = sort(scores, 'descend') ;
        matches = matches(:, perm) ;
        scores = scores(perm) ;
        
        %RANSAC
        feat_Ia= fa(1:2,matches(1,:));
        feat_Ia_normalized= normalise2dpts(makehomogeneous(feat_Ia));
        feat_Ib= fb(1:2,matches(2,:));
        feat_Ib_normalized= normalise2dpts(makehomogeneous(feat_Ib));
        [F, inliers]= ransacfitfundmatrix7(feat_Ia_normalized,feat_Ib_normalized,0.001);
        
        %display without ransac filter
%         clf ;
%         imagesc(cat(2, Ia, Ib)) ;
% 
%         xa = fa(1,matches(1,1:no_todisplay)) ;
%         xb = fb(1,matches(2,1:no_todisplay)) + size(Ia,2) ;
%         ya = fa(2,matches(1,1:no_todisplay)) ;
%         yb = fb(2,matches(2,1:no_todisplay)) ;
% 
%         hold on ;
%         h = line([xa ; xb], [ya ; yb]) ;
%         set(h,'linewidth', 1, 'color', 'b') ;
% 
%         vl_plotframe(fa(:,matches(1,1:no_todisplay))) ;
%         fb(1,:) = fb(1,:) + size(Ia,2) ;
%         vl_plotframe(fb(:,matches(2,1:no_todisplay))) ;
%         axis image off ;

        %display matches after ransac
        clf ;
        imagesc(cat(2, Ia, Ib)) ;

        xa = fa(1,matches(1,inliers(1,1:no_todisplay))) ;
        xb = fb(1,matches(2,inliers(1,1:no_todisplay))) + size(Ia,2) ;
        ya = fa(2,matches(1,inliers(1,1:no_todisplay))) ;
        yb = fb(2,matches(2,inliers(1,1:no_todisplay))) ;

        hold on ;
        h = line([xa ; xb], [ya ; yb]) ;
        set(h,'linewidth', 1, 'color', 'b') ;

        h1=vl_plotframe(fa(:,matches(1,inliers(1,1:no_todisplay)))) ;
        fb(1,:) = fb(1,:) + size(Ia,2) ;
        h2=vl_plotframe(fb(:,matches(2,inliers(1,1:no_todisplay)))) ;
        set(h1,'color','y','linewidth',2) ;
        set(h2,'color','y','linewidth',2) ;
        axis image off ;  
        
%         %mismatch
        outliers= setdiff(1:size(matches,2),inliers);
        out_xa = fa(1,matches(1,outliers(1,1:no_todisplay))) ;
        out_xb = fb(1,matches(2,outliers(1,1:no_todisplay)));
        out_ya = fa(2,matches(1,outliers(1,1:no_todisplay))) ;
        out_yb = fb(2,matches(2,outliers(1,1:no_todisplay))) ; 
        
        hold on ;
        h = line([out_xa ; out_xb], [out_ya ; out_yb]) ;
        set(h,'linewidth', 1, 'color', 'r') ;

        out_h1=vl_plotframe(fa(:,matches(1,outliers(1,1:no_todisplay)))) ;
        out_h2=vl_plotframe(fb(:,matches(2,outliers(1,1:no_todisplay)))) ;
        set(out_h1,'color','r','linewidth',2) ;
        set(out_h2,'color','r','linewidth',2) ;
        axis image off ;  
        pause
    
    end
end
