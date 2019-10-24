% function extract_keypoints(base_path)    
    base_path='/media/deep-three/deep_ssd2/Robotcar/2015-11-13-10-28-08';
    extrinsics_dir='/media/deep-three/Deep_Store/RobotCar_Images/extrinsics/';
    laser_dir= strcat(base_path,'/lms_front/');
    
    laser_timestamps = dlmread([base_path '/lms_front.timestamps']);
%     fid_locations=fopen(strcat(base_path,'pointcloud_locations_20m_10overlap.csv'), 'w');
%     %fprintf(fid_locations,'%s\t%s\t%s\t%s\t%s\t%s\n','timestamp','northing','easting', 'Ex','Ey','Ez');
%     fprintf(fid_locations,'%s,%s,%s\n','timestamp','northing','easting');
    
    laser_extrinisics = dlmread([extrinsics_dir 'lms_front.txt']);
    ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);
    G_ins_laser = SE3MatrixFromComponents(ins_extrinsics) \ SE3MatrixFromComponents(laser_extrinisics);
    count=0;
    target_pc_size=4096;
    
    %%%%%%for camera%%%%%%%%
    cam_files_location='/media/deep-three/Deep_Store/RobotCar_Images/extracted_data';
    folder=base_path(find(base_path=='/',1,'last'):end);
    camera_base_path=strcat(cam_files_location,folder,'/');
    camera='mono_right';

    if strcmp(camera, 'stereo')
        camera_path='stereo/centre/';
    else
        camera_path=strcat(camera,'/');
    end
    
    camera_timestamps=dlmread(strcat(camera_base_path,camera,'.timestamps'));
    camera_models_path='/media/deep-three/Deep_Store/RobotCar_Images/camera_models/';
    camera_extrinsics = dlmread(strcat(extrinsics_dir,camera,'.txt'));
    
    G_camera_ins = SE3MatrixFromComponents(camera_extrinsics) * SE3MatrixFromComponents(ins_extrinsics);
    [ fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel(camera_path,camera_models_path);
    camera_intrinsics=[fx,0,cx;0,fy,cy;0,0,1];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    no_tracked_frames=4;
    
    %for display
    colors={'k','r','b','y','g'};
    color_index=1;
    
    for chunk = 4:laser_timestamps(end,2)
       index_start=find(laser_timestamps(:,2) == chunk, 1, 'first');
       index_end= find(laser_timestamps(:,2) == chunk, 1, 'last');
       disp(strcat('Processing chunk: ',num2str(chunk),' Start Index: ',num2str(index_start),' End Index: ',num2str(index_end)));
       
       cam_index_start=find(camera_timestamps(:,2) == chunk, 1, 'first');
       cam_index_end= find(camera_timestamps(:,2) == chunk, 1, 'last');
       
       timestamps=laser_timestamps(index_start:index_end,1);
       cam_timestamps=camera_timestamps(cam_index_start:cam_index_end,1);
       
       if (chunk==1)
           %remove first few readings (in car park)
           timestamps=laser_timestamps(index_start+10000:index_end,1);
       end
       
       if (chunk==laser_timestamps(end,2))
           %remove first few readings (in car park)
           timestamps=laser_timestamps(index_start:index_end-1000,1);
       end
       
       global_poses=getGlobalPoses(strcat(base_path,'/gps/ins.csv'), timestamps');
       
       cam_global_poses=getGlobalPoses(strcat(camera_base_path,'/gps/ins.csv'), cam_timestamps');
       
       cam_frame_start=1;
       cam_frame_end=cam_frame_start+1;
       cam_start_next=cam_frame_start+1;
       
       frame_start= 1;
       frame_end=frame_start+1;
       frames=[frame_start];
       i=frame_start;
       j=i;
       start_next_frame=frame_start;
       got_next=0;
       while (frame_end<length(timestamps))
           %check if accumulated 10m movement
%            while(length(frames)<200 || ...
            
            %%%%%get start frame of camera 5m before the first lidar reading
            while(getDistance(cam_global_poses{cam_frame_start}(1,4),cam_global_poses{cam_frame_start}(2,4),global_poses{frame_start}(1,4), global_poses{frame_start}(2,4))>5)
                cam_frame_start=cam_frame_start+1;
            end
            %%%%%%
            
             while(getDistance(global_poses{i}(1,4), global_poses{i}(2,4),global_poses{frame_start}(1,4), global_poses{frame_start}(2,4))<20)
               
               if(j>(length(timestamps)-1))
                   break
               end  
               j=j+1;  
                 
               while((getDistance(global_poses{i}(1,4), global_poses{i}(2,4),global_poses{j}(1,4), global_poses{j}(2,4))<0.025)...
                       && (getRotation(global_poses{i}(1:3,1:3),global_poses{j}(1:3,1:3))*180/pi <30))
                    j=j+1;
                    if(j>(length(timestamps)-1))
                        break
                    end  
               end
               frames=[frames j];
               
%                %start next set 1 meter after
%                if length(frames)>=51
%                    start_next_frame=frames(1,51);
%                end
                if(j>(length(timestamps)-1))
                    break
                end
                
               if(getDistance(global_poses{frame_start}(1,4), global_poses{frame_start}(2,4),global_poses{j}(1,4), global_poses{j}(2,4))>10 && got_next==0)
                  start_next_frame=frames(1,end);
                  got_next=1;
                  
                  %%%%find where to start next cam
                  cam_start_next=cam_frame_start;            
                  while(getDistance(global_poses{j}(1,4),global_poses{j}(2,4),cam_global_poses{cam_start_next}(1,4), cam_global_poses{cam_start_next}(2,4))>5)
                     cam_start_next=cam_start_next+1;
                  end    
                  %%%%%%
                  
               end
                          
               i=j;
            end
           
           if(j>length(timestamps)-1)
               break
           end  
           frame_end=j;
           
           %%%%%%%find cam frame_end
           cam_frame_end=cam_start_next;
           while(getDistance(global_poses{j}(1,4),global_poses{j}(2,4),cam_global_poses{cam_frame_end}(1,4), cam_global_poses{cam_frame_end}(2,4))>1.0)
               cam_frame_end=cam_frame_end+1;
           end

           while(getDistance(global_poses{j}(1,4),global_poses{j}(2,4),cam_global_poses{cam_frame_end}(1,4), cam_global_poses{cam_frame_end}(2,4))<2.5)
               if(cam_frame_end>length(cam_timestamps))
                   break
               end               
               cam_frame_end=cam_frame_end+1;
           end                     
           %%%%%%%%
           
           pointcloud = [];
           for i=frames
                scan_path = [laser_dir num2str(timestamps(i,1)) '.bin'];
                scan_file = fopen(scan_path);
                scan = fread(scan_file, 'double');
                fclose(scan_file);

                scan = reshape(scan, [3 numel(scan)/3]);
                scan(3,:) = zeros(1, size(scan,2));

                scan = inv(global_poses{frame_start})*global_poses{i} * G_ins_laser * [scan; ones(1, size(scan,2))];
%                 scan = global_poses{i} * G_ins_laser * [scan; ones(1, size(scan,2))];
                pointcloud = [pointcloud scan(1:3,:)];
           end
           
           %save pointcloud
           origin_timestamp=timestamps(frames(1,1),1);

           [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.5);
           
           if (size(out_plane,1)<target_pc_size)
               if (got_next==0)
                   frame_start=frame_start+50;
               else
                   frame_start=start_next_frame;
               end
%                frame_start= frame_start+50;
               frame_end= frame_start+1;
%                frames=frames(1,frame_start:end);
               frames=[frame_start];
               i=frame_start;
               j=i;               
               got_next=0;
               disp('Faulty pointcloud');
              continue 
           end   
           %after road removal
            out_of_plane=pointcloud(:,out_plane);
            
            %flip z axis
            out_of_plane(3,:)=-out_of_plane(3,:);
            
%             figure(1);
%             pcshow(out_of_plane');
%             hold on;
%             pause;
            
            
            %%%%%%%%%%get images within a region
            cam_counter=cam_frame_start;
            cam_frames=[cam_frame_start];
            points=[];
            while (cam_counter<cam_frame_end)       
                while (size(cam_frames,2)<no_tracked_frames)
                    if(cam_counter>cam_frame_end)
                      break
                    end
                  
                    if((getDistance(cam_global_poses{cam_counter}(1,4), cam_global_poses{cam_counter}(2,4),...
                            cam_global_poses{cam_frames(end)}(1,4 ), cam_global_poses{cam_frames(end)}(2,4))<1.2)...
                                   && (getRotation(cam_global_poses{cam_counter}(1:3,1:3),cam_global_poses{cam_frames(end)}(1:3,1:3))*180/pi <10))
                        cam_counter=cam_counter+1; 
                    else
                        cam_frames=[cam_frames cam_counter];
                        rel_pose=global_poses{frame_start}\cam_global_poses{cam_counter};
%                         figure(1);
%                         scatter3(rel_pose(1,4),rel_pose(2,4),0,3.0,'r');
%                         hold on;
                        cam_counter=cam_counter+1;
                    end                         
                end
                
                disp('Got track');
                pause;
                
                if(cam_counter>cam_frame_end)
                  break
                end    
                
                if(size(cam_frames,2)~=no_tracked_frames)
                    disp("Error in number of tracked frames")
                    break
                end  
                
                %load and triangulate features
                images={};
                detected_features={};
                matches={};
                for k=1:no_tracked_frames
                    I = LoadImage(strcat(camera_base_path,camera_path), cam_timestamps(cam_frames(1,k),1), LUT);
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
    
                indexes={};
                %check for chain
                for a=1:(size(matches,2)-1)
                    indexes=ismember(matches{1,a+1}{1,1}(1,:),matches{1,a}{1,1}(2,:));
                    matches{1,a+1}{1,1}=matches{1,a+1}{1,1}(:,indexes);
                end

                for b=size(matches,2):-1:2
                    indexes=ismember(matches{1,b-1}{1,1}(2,:),matches{1,b}{1,1}(1,:));
                    matches{1,b-1}{1,1}=matches{1,b-1}{1,1}(:,indexes);
                end
    
                %construct chain
                tracks={};
                for a=1:size(matches{1,1}{1,1},2)
                    tracks{a}{1}=matches{1,1}{1,1}(1,a);
                    for b=1:no_tracked_frames-1
                        [r,c] = find(matches{1,b}{1,1}(1,:)==tracks{a}{b});
                        tracks{a}{b+1}=matches{1,b}{1,1}(2,c);
                    end
                end

                %construct camera matricesglobal_poses
                C={};
                for k=1:no_tracked_frames
%                     C{k} = camera_intrinsics * G_camera_ins(1:3,:) * (cam_global_poses{cam_frame_start}\cam_global_poses{cam_frames(1,k)});
                    C_rel=cam_global_poses{cam_frames(1,k)}\cam_global_poses{cam_frame_start};
                    C{k} = camera_intrinsics * C_rel(1:3,:);
%                     if(k==1)
%                         C{k} = camera_intrinsics * G_camera_ins(1:3,:);
%                     else
%                         C{k}= camera_intrinsics * G_camera_ins(1:3,:)*inv(cam_global_poses{cam_frames(1,1)})*cam_global_poses{cam_frames(1,k)};
%                     end
                end

                %construct 2xN array
                points=[];
                for j=1:size(tracks,2)
                    A=[];
                    for i=1:no_tracked_frames
                        x=detected_features{1,i}{1,1}(1:2,tracks{1,j}{1,i})';
                        A=[A;x];
                    end
                %points{size(points,2)+1}=A;
                    [pt, xy_reproj] = solvestereopt(A', C);
%                     pt = (global_poses{frame_start}\cam_global_poses{cam_frame_start})*inv(G_camera_ins)*pt;
                    points=[points, pt];
                end
                
                disp(num2str(size(points,2)));
                figure(1);
                scatter3(points(1,:),points(2,:),points(3,:),1,colors{color_index});
                color_index=color_index+1;
                axis([-30,30,-30,30,-30,30]);
                hold on;
                
%                 points = (global_poses{frame_start}\cam_global_poses{cam_frame_start})*inv(G_camera_ins)*points;
%                 scatter3(points(1,:),points(2,:),points(3,:),1,'k');
%                 hold on;

%                 %%%%%%%%%%%%%%DISPLAY MATCHES%%%%%%%%%%%%%%%%%%%%%%
                figure();
                image=images{1};
                for k=2:size(images,2)
                    image=cat(2,image,images{k});
                end
                imagesc(image);

%                 numtodisplay=20;
                for k=2:no_tracked_frames
                    fa=detected_features{k-1}{1};
                    da=detected_features{k-1}{2};
                    fb=detected_features{k}{1};
                    db=detected_features{k}{2};

%                     xa = fa(1,matches{1,k-1}{1,1}(1,1:numtodisplay)) + size(images{1},2)*(k-2);
%                     xb = fb(1,matches{1,k-1}{1,1}(2,1:numtodisplay)) + size(images{1},2)*(k-1) ;
%                     ya = fa(2,matches{1,k-1}{1,1}(1,1:numtodisplay));
%                     yb = fb(2,matches{1,k-1}{1,1}(2,1:numtodisplay));

%                     xa = fa(1,matches{1,k-1}{1,1}(1,:)) + size(images{1},2)*(k-2);
%                     xb = fb(1,matches{1,k-1}{1,1}(2,:)) + size(images{1},2)*(k-1) ;
%                     ya = fa(2,matches{1,k-1}{1,1}(1,:));
%                     yb = fb(2,matches{1,k-1}{1,1}(2,:)); 

%                     hold on ;
%                     h = line([xa ; xb], [ya ; yb]) ;
%                     set(h,'linewidth', 1, 'color', 'b') ;

                    if(k==2)
%                         h1=vl_plotframe(fa(:,matches{1,k-1}{1,1}(1,1:numtodisplay))) ;
                        h1=vl_plotframe(fa(:,matches{1,k-1}{1,1}(1,:))) ;                        
                        set(h1,'color','y','linewidth',2) ;
                    end    

                    fb(1,:) = fb(1,:) + size(images{1},2)*(k-1)  ;
%                     h2=vl_plotframe(fb(:,matches{1,k-1}{1,1}(2,1:numtodisplay))) ;
                    h2=vl_plotframe(fb(:,matches{1,k-1}{1,1}(2,:))) ;
                    set(h2,'color','y','linewidth',2) ;
                    axis image off ;              
                end
                pause;
%                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                %reset
                cam_frames=[cam_counter];
            end
            
            %transform points wrt ins of frame_start
%             points = inv(global_poses{frame_start})*cam_global_poses{cam_frames(1,1)}*inv(G_camera_ins)*points;
%             points = inv(G_camera_ins)*points;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            disp('Processed submap');
            pause
            
            figure(1);
            points = (global_poses{frame_start}\cam_global_poses{cam_frame_start})*inv(G_camera_ins)*points;
            scatter3(points(1,:),points(2,:),points(3,:),1,'k');
            
            disp('Shifted points');
            pause
            
            scale_size=1.001;
            downsampled=pcdownsample(pointCloud(out_of_plane'),'gridAverage',scale_size);
            
            while (downsampled.Count()<target_pc_size)
               scale_size=scale_size-0.025;
               if(scale_size<=0)
                    xyz=out_of_plane';
                    break;
               end
               downsampled=pcdownsample(pointCloud(out_of_plane'),'gridAverage',scale_size);
            end
            
            while (downsampled.Count()>target_pc_size)
               scale_size=scale_size+0.025;
               downsampled=pcdownsample(pointCloud(out_of_plane'),'gridAverage',scale_size);
            end
            
            if(scale_size>0)
                xyz=[downsampled.Location(:,1),downsampled.Location(:,2),downsampled.Location(:,3)];
            end            
            
            %force n to be target_pc_size (it is now less)
            num_extra_points=target_pc_size-size(xyz,1);
            permutation=randperm(length(out_of_plane));
            sample_out=permutation(1:num_extra_points);
            sample=out_of_plane(:,sample_out);%3xn            
            
            output=[xyz',sample];

%             scatter3(-out_of_plane(2,:),-out_of_plane(1,:),-out_of_plane(3,:),1,'b');
%             hold on;
%             scatter3(-points(2,:),-points(1,:),-points(3,:),3.0,'r');
%             pcshow(out_of_plane');
%             colormap(winter);
%             hold on;
%             pcshow(points(1:3,:)');
            
            
            %locate triangulated points
%             neighbors=[];
%             for i=1:size(points,2)
%                 [indices,dists] = findNeighborsInRadius(pointCloud(out_of_plane'),points(1:3,i)',5.0);
%                 neighbors=[neighbors;indices];
%             end 
%             
%             scatter3(-out_of_plane(2,:),-out_of_plane(1,:),-out_of_plane(3,:),1,'b');
%             hold on;
%             scatter3(-out_of_plane(2,neighbors),-out_of_plane(1,neighbors),-out_of_plane(3,neighbors),2,'r');
            
            pause
%            %transform wrt the centroid
%            x_cen=mean(output(1,:));
%            y_cen=mean(output(2,:));
%            z_cen=mean(output(3,:));
%            centroid=[x_cen;y_cen;z_cen;1];
%            centroid_g=double(global_poses{frame_start})*double(centroid);
%            
%            %sum of distances
%            sum=0;
%            for i=1:size(output,2)
%                sum=sum+sqrt((output(1,i)-x_cen)^2+(output(2,i)-y_cen)^2+(output(3,i)-z_cen)^2);
%            end
%            d=sum/size(output,2);
%            s=0.5/d;
%            
%            T=[[s,0,0,-s*(x_cen)];...
%                [0,s,0,-s*(y_cen)];...
%                [0,0,s,-s*(z_cen)];...
%                [0,0,0,1]];
%            scaled_output=T*[output; ones(1, size(output,2))];
%            scaled_output=-scaled_output;
% %           cleaned=scaled_output(1:3,:);
%            cleaned=[];
%            for i=1:size(scaled_output,2)
%                if(scaled_output(1,i)>=-1 && scaled_output(1,i)<=1 && scaled_output(2,i)>=-1 && scaled_output(2,i)<=1 ...
%                        && scaled_output(3,i)>=-1 && scaled_output(3,i)<=1)
%                     cleaned=[cleaned,scaled_output(:,i)];
%                end
%            end
%            
%            %make number of points equal to target_pc_size
%             num_extra_points=target_pc_size-size(cleaned,2);
%             disp(strcat(num2str(size(cleaned,2)),'.',num2str(num_extra_points)));
%             permutation=randperm(length(out_of_plane));
%             i=1;
%             while size(cleaned,2)<target_pc_size
%                new_point=-T*[out_of_plane(:,permutation(1,i));1];
%                if(new_point(1,1)>=-1 && new_point(1,1)<=1 && new_point(2,1)>=-1 && new_point(2,1)<=1 ...
%                        && new_point(3,1)>=-1 && new_point(3,1)<=1)                
%                     cleaned=[cleaned,new_point];
%                end
%                i=i+1;
%             end
%             cleaned=cleaned(1:3,:);
%             
%             if(size(cleaned,2)~=target_pc_size)
%                frame_start=start_next_frame;
% %                frame_start= frame_start +50;
%                frame_end= frame_start+1;
% %                frames=frames(1,frame_start:end);index_start
%                frames=[frame_start];
%                i=frame_start;
%                j=i;
%                disp('Invalid pointcloud')
%                continue;
%             end
            
%             fileID = fopen(strcat(base_path,'pointcloud_20m_10overlap/', num2str(origin_timestamp),'.bin'),'w');
%             fwrite(fileID,cleaned,'double');
%             fclose(fileID);
%             disp(strcat(num2str(size(xyz,1)),'.',num2str(size(cleaned,2))));
%             
%             %%%%OUTPUT TO CSV%%%%%%%%
%             fprintf(fid_locations, '%s,%f,%f\n',num2str(origin_timestamp),centroid_g(1,1), centroid_g(2,1));
            
%             count=count+1;            
%             if mod(count,10)==1
% %                 h=figure('name',strcat(num2str(centroid_g(1,1)),'.',num2str(centroid_g(2,1))));
% %                 h=figure('name',strcat(num2str(centroid_g(1,1)),'.',num2str(centroid_g(2,1))));
% %                 colormap(winter);
% %                 scatter3(-pointcloud(2,out_plane),-pointcloud(1,out_plane),-pointcloud(3,out_plane),1);
% %                 pc_toshow=-[pointcloud(2,out_plane);pointcloud(1,out_plane);pointcloud(3,out_plane)];
% %                 pcshow(pc_toshow');
% %                 hold on;
% %                 scatter3(-pointcloud(2,in_plane),-pointcloud(1,in_plane),-pointcloud(3,in_plane),1,'r');
% %                 colormap(winter);
% %                 scatter3(-pointcloud(2,in_plane),-pointcloud(1,in_plane),-pointcloud(3,in_plane),1);
% %                 pcshow(-pointcloud');
% %                 axis equal;
% %                 pause;
% %                 if(centroid_g(1,1)<x_threshold && centroid_g(2,1)<y_threshold)
% %                     scatter3(cleaned(2,:),cleaned(1,:),cleaned(3,:),1,'g')
% %                 else
% %                     scatter3(cleaned(2,:),cleaned(1,:),cleaned(3,:),1,'b')
% %                 end
% %                 h=figure('name',strcat(num2str(centroid_g(1,1)),'.',num2str(centroid_g(2,1))));
%                 g=figure('name',strcat(num2str(chunk),'.',num2str(origin_timestamp)));
%                 c=[-cleaned(2,:);cleaned(1,:);cleaned(3,:)];
%                 pcshow(c','MarkerSize',50);
%                 colormap(winter);
% %                 scatter3(cleaned(2,:),cleaned(1,:),cleaned(3,:),1);
%                 axis equal;
% %                 hold_on;
%                 pause;
% %                 set(h,'visible','on');
%             end
            
            
           %reset
           if (got_next==0)
               frame_start=frame_start+50;
               cam_frame_start=cam_start_next;

           else
               frame_start=start_next_frame;
               cam_frame_start=cam_frame_start+10;
           end
           
           cam_start_next=cam_frame_start+1;
           cam_frame_end=cam_frame_start+1;
           
%            frame_start= frame_end;
           frame_end= frame_start+1;
%            frames=frames(1,frame_start:end);
           frames=[frame_start];
           i=frame_start;
           j=i;
           got_next=0;
           
       end
    
    end
%     fclose(fid_locations);
%     plot_pointcloud_path(base_path)
% end

    