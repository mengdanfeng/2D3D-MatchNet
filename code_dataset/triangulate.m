base_path='/media/deep-three/Deep_Store/RobotCar_Images/extracted_data/2014-05-19-13-05-38/';
camera='stereo';

if strcmp(camera, 'stereo')
    camera_path='stereo/centre/';
else
    camera_path=strcat(camera,'/');
end

timestamps=dlmread(strcat(base_path,camera,'.timestamps'));
ins_file=strcat(base_path,'gps/ins.csv');

camera_models_path='/media/deep-three/Deep_Store/RobotCar_Images/camera_models/';

extrinsics_dir='/media/deep-three/Deep_Store/RobotCar_Images/extrinsics/';
camera_extrinsics = dlmread(strcat(extrinsics_dir,camera,'.txt'));
ins_extrinsics= dlmread(strcat(extrinsics_dir,'ins.txt'));

laser_dir='/media/deep-three/deep_ssd2/Robotcar/2014-05-19-13-05-38/lms_front/';

G_camera_ins = SE3MatrixFromComponents(camera_extrinsics) * SE3MatrixFromComponents(ins_extrinsics);

%get LUT
[ fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel(camera_path,camera_models_path);
camera_intrinsics=[fx,0,cx;0,fy,cy;0,0,1];

no_tracked_frames=4;
prev=-1;
frame_start=1500;
frames=[frame_start];
start_next=frame_start+10;

prev=frame_start;
j=frame_start+1;
frame_poses={};
while(j<size(timestamps,1))
    while(size(frames,2)<no_tracked_frames)
        
        if(j>size(timestamps,1))
           break 
        end
        
        poses=InterpolatePoses(ins_file, [timestamps(prev,1),timestamps(j,1)], timestamps(frame_start,1));
        
%         if(getDistance(poses{2}(1,4), poses{2}(2,4),0, 0)>10)
%             start_next=j;
%         end
        
        if((getDistance(poses{1}(1,4), poses{1}(2,4),poses{2}(1,4 ), poses{2}(2,4))<1.2)...
                       && (getRotation(poses{1}(1:3,1:3),poses{2}(1:3,1:3))*180/pi <10))
            j=j+1; 
        else
            frames=[frames j];
            frame_poses{size(frames,2)-1}=poses{2};
            prev=j;
            j=j+1;
        end
        
    end
    
    if(size(frames,2)~=no_tracked_frames)
        disp("Error in number of tracked frames")
        break
    end    
    
    images={};
    detected_features={};
    matches={};
    for k=1:no_tracked_frames
        I = LoadImage(strcat(base_path,camera_path), timestamps(frames(1,k),1), LUT);
        I_gray= single(rgb2gray(I));
        [f,d]=vl_sift(I_gray);
        detected_features{k}={f,d};
        
        images{k}=I;
        
        if(k~=1)            
            fa=detected_features{k-1}{1};
            da=detected_features{k-1}{2};
            fb=detected_features{k}{1};
            db=detected_features{k}{2};

            [cur_matches, cur_scores] = vl_ubcmatch(da,db) ;
            [drop, perm] = sort(cur_scores, 'descend') ;
            cur_matches = cur_matches(:, perm) ;
            cur_scores = cur_scores(perm) ;

            %RANSAC
            feat_Ia= fa(1:2,cur_matches(1,:));
            feat_Ia_normalized= normalise2dpts(makehomogeneous(feat_Ia));
            feat_Ib= fb(1:2,cur_matches(2,:));
            feat_Ib_normalized= normalise2dpts(makehomogeneous(feat_Ib));
            [F, inliers]= ransacfitfundmatrix7(feat_Ia_normalized,feat_Ib_normalized,0.01);            
            
            matches{k-1}={cur_matches(:,inliers),cur_scores(:,inliers)};
        end
    end
    
%     indexes={};
%     %check for chain
%     for a=1:(size(matches,2)-1)
%         indexes=ismember(matches{1,a+1}{1,1}(1,:),matches{1,a}{1,1}(2,:));
%         matches{1,a+1}{1,1}=matches{1,a+1}{1,1}(:,indexes);
%     end
%     
%     for b=size(matches,2):-1:2
%         indexes=ismember(matches{1,b-1}{1,1}(2,:),matches{1,b}{1,1}(1,:));
%         matches{1,b-1}{1,1}=matches{1,b-1}{1,1}(:,indexes);
%     end
%     
%     %construct chain
%     tracks={};
%     for a=1:size(matches{1,1}{1,1},2)
%         tracks{a}{1}=matches{1,1}{1,1}(1,a);
%         for b=1:no_tracked_frames-1
%             [r,c] = find(matches{1,b}{1,1}(1,:)==tracks{a}{b});
%             tracks{a}{b+1}=matches{1,b}{1,1}(2,c);
%         end
%     end
%     
%     %construct camera matrices
%     C={};
%     for k=1:no_tracked_frames
%         if(k==1)
%             C{k} = camera_intrinsics * G_camera_ins(1:3,:);
%         else
%             C{k}= camera_intrinsics * G_camera_ins(1:3,:) * frame_poses{k-1};
%         end
%     end

%     %construct 2xN array
%     points=[];
%     for j=1:size(tracks,2)
%         A=[];
%         for i=1:no_tracked_frames
%             x=detected_features{1,i}{1,1}(1:2,tracks{1,j}{1,i})';
%             A=[A;x];
%         end
% %         points{size(points,2)+1}=A;
%         [pt, xy_reproj] = solvestereopt(A', C);
%         points=[points, pt];
%     end
%     
%     %transform points wrt ins of frame_start
%     points = G_camera_ins \ points;
    
%     %points wrt to ins of frame_start
%     [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
%     extrinsics_dir, timestamps(frames(1,1),1)-1e7, timestamps(frames(1,no_tracked_frames),1)+1e7, timestamps(frames(1,1),1));
% 
%     %remove road before getting neighbors
%     [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.5);
%     pointcloud=pointcloud(:,out_plane);
% 
%     %find near lidar points for each triangulated point
%     neighbors=[];
%     for i=1:size(points,2)
%         [indices,dists] = findNeighborsInRadius(pointCloud(pointcloud'),points(1:3,i)',3.0);
%         neighbors=[neighbors;indices];
%     end    
    
%     scatter3(-pointcloud(2,:),-pointcloud(1,:),-pointcloud(3,:),1,'b');
%     hold on;
%     scatter3(points(2,:),points(1,:),points(3,:),3.0,'r');
%     pause;
    
%     %display matches
    image=images{1};
    for k=2:size(images,2)
        image=cat(2,image,images{k});
    end
    imagesc(image);
    
    for k=2:no_tracked_frames
        fa=detected_features{k-1}{1};
        da=detected_features{k-1}{2};
        fb=detected_features{k}{1};
        db=detected_features{k}{2};
        
        xa = fa(1,matches{1,k-1}{1,1}(1,:)) + size(images{1},2)*(k-2);
        xb = fb(1,matches{1,k-1}{1,1}(2,:)) + size(images{1},2)*(k-1) ;
        ya = fa(2,matches{1,k-1}{1,1}(1,:));
        yb = fb(2,matches{1,k-1}{1,1}(2,:));
        hold on ;
        h = line([xa ; xb], [ya ; yb]) ;
        set(h,'linewidth', 1, 'color', 'b') ;

        if(k==2)
            h1=vl_plotframe(fa(:,matches{1,k-1}{1,1}(1,:))) ;
            set(h1,'color','y','linewidth',2) ;
        end    
        
        fb(1,:) = fb(1,:) + size(images{1},2)*(k-1)  ;
        h2=vl_plotframe(fb(:,matches{1,k-1}{1,1}(2,:))) ;        
        set(h2,'color','y','linewidth',2) ;
        axis image off ;              
    end 
    
    pause
    
    %reset variables
    start_next=frames(1,no_tracked_frames)+5;
    frame_start=start_next;
    frames=[frame_start];
    frame_poses={};

    prev=frame_start;
    j=frame_start+1;   
    
end

