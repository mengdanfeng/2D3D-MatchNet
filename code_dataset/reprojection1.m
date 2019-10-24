close all
clear all
clc

% load params
loadParams

for chunk = start_chunk:laser_timestamps(end,2)
    %find readings in chunk
    laser_index_start= find(laser_timestamps(:,2) == chunk, 1, 'first');
    laser_index_end= find(laser_timestamps(:,2) == chunk, 1, 'last');
    camera_index_start= find(camera_timestamps(:,2) == chunk, 1, 'first');
    camera_index_end= find(camera_timestamps(:,2) == chunk, 1, 'last');    
    
    l_timestamps=laser_timestamps(laser_index_start:laser_index_end,1);
    c_timestamps=camera_timestamps(camera_index_start:camera_index_end,1);
    
    disp(strcat('Processing chunk: ',num2str(chunk),' Laser Start Index: ',num2str(laser_index_start),' Laser End Index: ',num2str(laser_index_end)));
    %filter edge cases
    if (chunk==1)
       %remove first few readings (in car park)
       l_timestamps=laser_timestamps(laser_index_start+5000:laser_index_end,1);
       c_timestamps=camera_timestamps(camera_index_start+200:camera_index_end,1);
    end
    
    if (chunk==laser_timestamps(end,2))
       %remove last readings
       l_timestamps=laser_timestamps(laser_index_start:laser_index_end-1000,1);
    end
    
    %%%%%%%%%%POSES%%%%%%%%%%
    laser_global_poses=getGlobalPoses(strcat(base_path,'/gps/ins.csv'), l_timestamps');
    disp(strcat('Processing chunk: ',num2str(chunk),' Loaded laser poses'));
    cam_global_poses=getGlobalPoses(strcat(camera_base_path,'/gps/ins.csv'), c_timestamps');
    disp(strcat('Processing chunk: ',num2str(chunk),' Loaded camera poses'));
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%Counter Variables%%%%
    %laser
    frame_start=1;
    frame_end=frame_start+1;
    frames=[];
    i=frame_start;
    j=i;
    start_next_frame=frame_start;
    got_next=0;
    
    %camera
    cam_frame_start=1;
    cam_frame_end=cam_frame_start+1;
    cam_start_next=cam_frame_start+1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%
 
    while(frame_end<length(l_timestamps))
    tic
        %%%%%find cam_frame_start Xm before first lidar scan
        while(getDistance1(cam_global_poses{cam_frame_start}, laser_global_poses{frame_start}) >dist_cam_laser)
            cam_frame_start=cam_frame_start+1
        end        
        %%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%GET FRAMES TO GENERATE SUBMAP%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        while(getDistance1(laser_global_poses{i},laser_global_poses{frame_start})<submap_cover_distance)
            if(j>(length(l_timestamps)-1))
               break
            end  
            j=j+1;  

            while((getDistance1(laser_global_poses{i}, laser_global_poses{j})<laser_reading_distance)...
                   && (getRotation(laser_global_poses{i}(1:3,1:3), laser_global_poses{j}(1:3,1:3))*180/pi <laser_reading_angle))
                j=j+1;
                if(j>(length(l_timestamps)-1))
                    break
                end  
            end
            frames=[frames j];

            if(j>(length(l_timestamps)-1))
                break
            end

            if(getDistance(laser_global_poses{frame_start}(1,4), laser_global_poses{frame_start}(2,4), laser_global_poses{j}(1,4), laser_global_poses{j}(2,4))>dist_start_next_frame && got_next==0)
              start_next_frame=frames(1,end);
              got_next=1;

              %%%%find cam_start_next
              cam_start_next=cam_frame_start;            
              while(getDistance(laser_global_poses{j}(1,4),laser_global_poses{j}(2,4),cam_global_poses{cam_start_next}(1,4), cam_global_poses{cam_start_next}(2,4))>dist_cam_laser)
                 cam_start_next=cam_start_next+1;
              end    
              %%%%%%
            end
            i=j;
        end
        toc
% 
%         if(j>length(l_timestamps)-1)
%             break
%         end  
%         frame_end=j;        
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         %%%%%%%find cam_frame_end
%         cam_frame_end=cam_start_next;
%         while(getDistance(laser_global_poses{j}(1,4),laser_global_poses{j}(2,4),cam_global_poses{cam_frame_end}(1,4), cam_global_poses{cam_frame_end}(2,4))>1.0)
%            cam_frame_end=cam_frame_end+1;
%         end
% 
%         while(getDistance(laser_global_poses{j}(1,4),laser_global_poses{j}(2,4),cam_global_poses{cam_frame_end}(1,4), cam_global_poses{cam_frame_end}(2,4))<2.5)
%            if(cam_frame_end>length(c_timestamps))
%                break
%            end               
%            cam_frame_end=cam_frame_end+1;
%         end                     
%         %%%%%%%% 
%         
%         %%%%%%%Build Pointcloud%%%%%%%
%         pointcloud = [];
%         for i=frames
%             scan_path = [laser_dir num2str(l_timestamps(i,1)) '.bin'];
%             scan_file = fopen(scan_path);
%             scan = fread(scan_file, 'double');
%             fclose(scan_file);
% 
%             scan = reshape(scan, [3 numel(scan)/3]);
%             scan(3,:) = zeros(1, size(scan,2));
% 
%             scan = inv(laser_global_poses{frame_start})*laser_global_poses{i} * G_ins_laser * [scan; ones(1, size(scan,2))];
%             pointcloud = [pointcloud scan(1:3,:)];
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         %%%Remove ground plane
%         [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.5);
%         out_of_plane=pointcloud(:,out_plane);
%         
%         %%%Find the normals and curvature
%         [normals,curvature] = findPointNormals(out_of_plane',[],[0,0,10],true);
%         coloursFront = (curvature - min(curvature)) / ...
%                (max(curvature) - min(curvature));
%         coloursFront = 1 ./ (1 + exp(-10*(coloursFront - mean(coloursFront))));
%         pc_keypoints = find(coloursFront > 0.5);
%         
%         figure(2),
%         scatter3(-out_of_plane(2,1:end), -out_of_plane(1,1:end),...
%         -out_of_plane(3,1:end), 1, coloursFront(1:end), '.');
%         hold on
%         idx_disp = find(coloursFront > 0.5);
%         scatter3(-out_of_plane(2,idx_disp), -out_of_plane(1,idx_disp),...
%         -out_of_plane(3,idx_disp), 5, 'r.');
%         hold off
%         axis equal
%         
%         %%%%%%%%%%%Check if not enough points after road removal
%         if (size(out_plane,1)<target_pc_size)
%             %reset variables
%             if (got_next==0)
%                frame_start=frame_start+50;
%                cam_frame_start=cam_frame_start+7;
%             else
%                frame_start=start_next_frame;
%                cam_frame_start=cam_start_next;
%             end
%             frame_end= frame_start+1;
%             frames=[frame_start];
%             i=frame_start;
%             j=i;               
%             got_next=0;
%             
%             disp('Faulty pointcloud');
%             continue 
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Process Images within pointcloud range%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %counter variables
%         cam_counter=cam_frame_start;
%         cam_frames=[cam_frame_start];
%         
%         %%%Get spaced frames
%         while(cam_counter<cam_frame_end)
%             if((getDistance(cam_global_poses{cam_counter}(1,4), cam_global_poses{cam_counter}(2,4),...
%                     cam_global_poses{cam_frames(end)}(1,4 ), cam_global_poses{cam_frames(end)}(2,4))<camera_reading_distance)...
%                            && (getRotation(cam_global_poses{cam_counter}(1:3,1:3),cam_global_poses{cam_frames(end)}(1:3,1:3))*180/pi <camera_reading_angle))
%                 cam_counter=cam_counter+1; 
%             else
%                 cam_frames=[cam_frames cam_counter];
%                 cam_counter=cam_counter+1;
%             end   
%         end
%         
%         %%%%%%%%%%%%%%%%%%%%%Compute Features%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         images={};
%         detected_features={};
%         
%         for i=1:size(cam_frames,2)
%             I = LoadImage(strcat(camera_base_path,camera_dir), c_timestamps(cam_frames(1,i),1), LUT);
%             I_gray= single(rgb2gray(I));
%             [f,d]=vl_sift(I_gray);
%             detected_features{i}={f,d};
%             images{i}=I;
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         disp(strcat('Computed features:',num2str(size(cam_frames,2)),' frames.'));
% %         pause;
%         
%         %%%%%%%%%%%%%%%Project pointcloud into camera frame%%%%%%%%%%%%%%%%
%         pc_img_projs={};
%         for i=1:size(cam_frames,2)
%             image= images{i};
% 
%             cur_frame_pose= cam_global_poses{cam_frames(1,i)};
%             rel_pose= inv(cur_frame_pose) * laser_global_poses{frame_start};
% %             transformed_pointcloud= rel_pose * [pointcloud; ones(1, size(pointcloud,2))];
%             transformed_pointcloud= rel_pose * [out_of_plane; ones(1, size(out_of_plane,2))];
%             
%             %Transform pointcloud wrt camera image frame
%             xyz = (G_camera_image \ G_camera_ins ...
%               * transformed_pointcloud).';
%             xyz(:,4) = [];
%             
%             %Project points into image
%             uv = [ fx .* xyz(:,1) ./ xyz(:,3) + cx, ...
%                  fy .* xyz(:,2) ./ xyz(:,3) + cy];
%              
%             %Get indices of points that are projected into image
%             in_front = xyz(:,3) >= 0;             
%             in_img = (uv(:,1) >= 0.5 & uv(:,1) < size(image,2)-0.5) & (uv(:,2) >= 0.5 & uv(:,2) < size(image,1)-0.5);
%             logical_idx= and(in_front,in_img);
%             idx_in_img= find(logical_idx==1);
%            
% %             pc_img_proj= ones(size(pointcloud,2),2)*-2000;
%             pc_img_proj= ones(size(out_of_plane,2),2)*-2000;
%             pc_img_proj(idx_in_img,:)= uv(idx_in_img,:);
%             pc_img_projs{i}= pc_img_proj;
% 
%             if sum(idx_in_img) == 0
%                 warning('No points project into image. Is the vehicle stationary?');
%             elseif sum(idx_in_img) < 1000
%                 warning('Very few points project into image. Is the vehicle stationary?');
%             end
% 
%             uv = uv(idx_in_img,:);
%             colours = xyz(idx_in_img, 3);
%             
%             %%DISPLAY
% %             figure(1);
% %             clf;
% %             subplot(1,2,1);
% %             imshow(image);
% %             subplot(1,2,2);
% %             imshow(image);
% %             colormap jet;
% %             hold on;
% %             scatter(uv(:,1),uv(:,2), 90, colours, '.');                
% %             pause
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
%         
% %         %%%%%%%Get matches EXPENSIVE.. TODO? 
% %         matches={};
% %         for i=1:(size(cam_frames,2)-1)
% %             fa=detected_features{i}{1};
% %             da=detected_features{i}{2};
% %             fb=detected_features{i+1}{1};
% %             db=detected_features{i+1}{2};
% %             
% %             [cur_matches, cur_scores] = vl_ubcmatch(da,db) ;
% %             [drop, perm] = sort(cur_scores, 'descend') ;
% %             cur_matches = cur_matches(:, perm) ;
% %             cur_scores = cur_scores(perm) ;
% %             feat_Ia= fa(1:2,cur_matches(1,:));
% %             feat_Ia_normalized= normalise2dpts(makehomogeneous(feat_Ia));
% %             feat_Ib= fb(1:2,cur_matches(2,:));
% %             feat_Ib_normalized= normalise2dpts(makehomogeneous(feat_Ib));
% %             [F, inliers]= ransacfitfundmatrix7(feat_Ia_normalized,feat_Ib_normalized,0.01);            
% % 
% %             matches{i}=cur_matches(:,inliers);
% %         end
% %         disp(strcat('Computed Matches:',num2str(size(cam_frames,2)),' frames.'));
% %         
% %         %%%Get tracks
% %         tracks={};
% %         for i=1:size(matches,2)
% %             %%%
% %         end 
% %         %%%%%%%%%%%        
% 
% 
%         %%%%%%%%%%%%%Get Matches and Corresponding Lidar Point%%%%%%%%%%%%%
% %         submap_mask=zeros(size(pointcloud,2),1);
%         submap_mask=zeros(size(out_of_plane,2),1);
%         keypoints_idx=[];
%         
%         for i=1:(size(cam_frames,2)-no_tracked_frames+1)
%             matches={};
%             for k=(1:no_tracked_frames-1)          
%                 fa=detected_features{i+k-1}{1};
%                 da=detected_features{i+k-1}{2};
%                 fb=detected_features{i+k}{1};
%                 db=detected_features{i+k}{2};
% 
%                 [cur_matches, cur_scores] = vl_ubcmatch(da,db) ;
%                 [drop, perm] = sort(cur_scores, 'descend') ;
%                 cur_matches = cur_matches(:, perm) ;
%                 cur_scores = cur_scores(perm) ;
% 
%                 %RANSAC
%                 feat_Ia= fa(1:2,cur_matches(1,:));
%                 feat_Ia_normalized= normalise2dpts(makehomogeneous(feat_Ia));
%                 feat_Ib= fb(1:2,cur_matches(2,:));
%                 feat_Ib_normalized= normalise2dpts(makehomogeneous(feat_Ib));
%                 [F, inliers]= ransacfitfundmatrix7(feat_Ia_normalized,feat_Ib_normalized,0.01);            
% 
%                 matches{k}={cur_matches(:,inliers),cur_scores(:,inliers)};
%             end
%          
%             indexes={};
%             %Check for chain
%             for a=1:(size(matches,2)-1)
%                 indexes=ismember(matches{1,a+1}{1,1}(1,:),matches{1,a}{1,1}(2,:));
%                 matches{1,a+1}{1,1}=matches{1,a+1}{1,1}(:,indexes);
%             end
% 
%             for b=size(matches,2):-1:2
%                 indexes=ismember(matches{1,b-1}{1,1}(2,:),matches{1,b}{1,1}(1,:));
%                 matches{1,b-1}{1,1}=matches{1,b-1}{1,1}(:,indexes);
%             end
% 
%             %Construct chain
%             tracks={};
%             for a=1:size(matches{1,1}{1,1},2)
%                 tracks{a}{1}=matches{1,1}{1,1}(1,a);
%                 for b=1:no_tracked_frames-1
%                     [r,c] = find(matches{1,b}{1,1}(1,:)==tracks{a}{b});
%                     tracks{a}{b+1}=matches{1,b}{1,1}(2,c);
%                 end
%             end
%             tracks_mat=cell2mat(cellfun(@(x) cell2mat(x),tracks,'un',0));
%             tracks_mat=reshape(tracks_mat,4,size(tracks_mat,2)/4)';
%             disp(strcat('Computed tracks'));
%             
%             
%             %%Search Window Width
%             for t=1:size(tracks_mat,1)
%                 %Corresponding locations of matchs in image coordinates
%                 xy=[];
% %                 in_sift_region=ones(size(pointcloud,2),1);
%                 in_sift_region=ones(size(out_of_plane,2),1);
%                 for k=1:no_tracked_frames
%                     fa=detected_features{i+k-1}{1};
%                     xy=[xy; [fa(1,tracks_mat(t,k)), fa(2,tracks_mat(t,k))]];
%                     
%                     %Get window boundaries
%                     xborder= [fa(1,tracks_mat(t,k))-window_width; fa(1,tracks_mat(t,k))+window_width; fa(1,tracks_mat(t,k))+window_width; fa(1,tracks_mat(t,k))-window_width];
%                     yborder= [fa(2,tracks_mat(t,k))+window_width; fa(2,tracks_mat(t,k))+window_width; fa(2,tracks_mat(t,k))-window_width; fa(2,tracks_mat(t,k))-window_width];
%                     
%                     %Points near the sift feature
%                     in= inpolygon(pc_img_projs{i+k-1}(:,1),pc_img_projs{i+k-1}(:,2),xborder, yborder);
%                     in_sift_region= and(in_sift_region, in);
%                 end
%                 
%                 %Find optimal point among points within the region
%                 in_pc=find(in_sift_region==1);
%                 dist=zeros(size(in_pc,1),1);
%                 for k=1:no_tracked_frames
%                    dist= dist+sqrt((pc_img_projs{i+k-1}(in_pc,1)-xy(k,1)).^2 + (pc_img_projs{i+k-1}(in_pc,2)-xy(k,2)).^2); 
%                 end
%                 
%                 %Get index value of point with minimum sum dist in tracked frames
%                 [v,idx]=min(dist);
%                 idx= in_pc(idx);
%                 
%                 if(size(idx,1)==0)
%                     continue
%                 end
%                 
%                 %Check if reprojection
%                 [indices,distances] = findNearestNeighbors(pointCloud(out_of_plane'),out_of_plane(:,idx)',1);
% %                 pause
%                 
% %                 if ismember(idx, pc_keypoints)
%                 if(distances < 0.5)
%                     %Detected keypoints
%                     idx = indices;
%                     
%                     if(ismember(idx, keypoints_idx))
%                         continue
%                     end
%                     
%                     keypoints_idx= [keypoints_idx; idx];
% 
%                     %Get weight through normal distribution
% %                     X = pointcloud(:,in_pc)';
% %                     mu = pointcloud(:,idx)';
%                     X = out_of_plane(:,in_pc)';
%                     mu = out_of_plane(:,idx)';
%                     s = [1.0,0,0; 0,1.0,0; 0,0,1.0];
%                     weights= mvnpdf(X, mu, s);
%                     submap_mask(in_pc)=max(submap_mask(in_pc), weights); 
%                 end
% 
% %                 %%Display
% %                 figure(2);
% %                 clf
% %                 subplot(2,1,1);
% %                 image=images{i};
% %                 for k=2:no_tracked_frames
% %                     image=cat(2,image,images{i+k});
% %                 end
% %                 imagesc(image);                
% %                 for k=1:no_tracked_frames
% %                     fa=detected_features{i+k-1}{1};
% %                     da=detected_features{i+k-1}{2};
% % 
% %                     hold on ;
% %                     fa(1,:) = fa(1,:) + size(images{i},2)*(k-1)  ;
% %                     h2=vl_plotframe(fa(:,tracks_mat(t,k))) ;
% %                     set(h2,'color','y','linewidth',2) ;
% %                     axis image off ;
% %                 end                
%                 
% %                 subplot(2,1,2);           
% %                 pcshow(pointcloud');
% %                 hold on;
% % %                 in_pc=find(in_sift_region==1);
% % %                 scatter3(pointcloud(1,in_pc),pointcloud(2,in_pc),pointcloud(3,in_pc),1,'g');
% % %                 hold on;
% %                 scatter3(pointcloud(1,idx),pointcloud(2,idx),pointcloud(3,idx),10,'r');
% %                 disp('Get Image Locations');
% %                 pause
%             end
%             
% %             %%%%DISPLAY POINTCLOUD%%%%%%
% %             figure(3);
% % %             clf
% %             subplot(ceil((size(cam_frames,2)-no_tracked_frames+1)/3),3,i);
% %             image=images{i};
% %             for k=2:no_tracked_frames
% %                 image=cat(2,image,images{i+k-1});
% %             end
% %             imagesc(image);                
% %             for k=1:no_tracked_frames
% %                 fa=detected_features{i+k-1}{1};
% %                 da=detected_features{i+k-1}{2};
% % 
% %                 hold on ;
% %                 fa(1,:) = fa(1,:) + size(images{i},2)*(k-1)  ;
% %                 h2=vl_plotframe(fa(:,tracks_mat(:,k))) ;
% %                 set(h2,'color','y','linewidth',2) ;
% %                 axis image off ;
% %             end               
% %             
% %             subplot(2,1,2);
% %             colormap jet;
% %             colours= pc_mask;
% %             pcshow(pointcloud',colours);
% %             disp(strcat('Constructed heat map'));
% %             axis equal;
% %             pause            
%             
% %             %%%%%DISPLAY FEATURE MATCHES%%%%
% %             figure();
% %             image=images{i};
% %             for k=2:no_tracked_frames
% %                 image=cat(2,image,images{i+k});
% %             end
% %             imagesc(image);
% %                 
% %             numtodisplay=20;
% %             for k=2:no_tracked_frames
% %                 fa=detected_features{i+k-1}{1};
% %                 da=detected_features{i+k-1}{2};
% %                 fb=detected_features{i+k}{1};
% %                 db=detected_features{i+k}{2};
% % 
% %                 xa = fa(1,matches{1,k-1}{1,1}(1,1:numtodisplay)) + size(images{i},2)*(k-2);
% %                 xb = fb(1,matches{1,k-1}{1,1}(2,1:numtodisplay)) + size(images{i},2)*(k-1) ;
% %                 ya = fa(2,matches{1,k-1}{1,1}(1,1:numtodisplay));
% %                 yb = fb(2,matches{1,k-1}{1,1}(2,1:numtodisplay));
% % 
% % %                 xa = fa(1,matches{1,k-1}{1,1}(1,:)) + size(images{1},2)*(k-2);
% % %                 xb = fb(1,matches{1,k-1}{1,1}(2,:)) + size(images{1},2)*(k-1) ;
% % %                 ya = fa(2,matches{1,k-1}{1,1}(1,:));
% % %                 yb = fb(2,matches{1,k-1}{1,1}(2,:)); 
% % 
% %                 hold on ;
% %                 h = line([xa ; xb], [ya ; yb]) ;
% %                 set(h,'linewidth', 1, 'color', 'b') ;
% % 
% %                 if(k==2)
% %                     h1=vl_plotframe(fa(:,matches{1,k-1}{1,1}(1,1:numtodisplay))) ;
% % %                     h1=vl_plotframe(fa(:,matches{1,k-1}{1,1}(1,:))) ;                        
% %                     set(h1,'color','y','linewidth',2) ;
% %                 end    
% % 
% %                 fb(1,:) = fb(1,:) + size(images{i},2)*(k-1)  ;
% %                 h2=vl_plotframe(fb(:,matches{1,k-1}{1,1}(2,1:numtodisplay))) ;
% % %                 h2=vl_plotframe(fb(:,matches{1,k-1}{1,1}(2,:))) ;
% %                 set(h2,'color','y','linewidth',2) ;
% %                 axis image off ;
% %             end
% %             pause;
% %             %%%%%%%%%%%%%%%%
%         end
%         
%         %%%%DISPLAY SUBMAP HEATMAP%%%%
%         figure(4);
%         clf
%         colormap jet;
%         colours= submap_mask;
% %         pcshow(pointcloud',colours);
% %         hold on;
% %         scatter3(pointcloud(1,keypoints_idx),pointcloud(2,keypoints_idx),pointcloud(3,keypoints_idx),10,'gh');
%         pcshow(out_of_plane',colours);
%         hold on;
%         scatter3(out_of_plane(1,keypoints_idx),out_of_plane(2,keypoints_idx),out_of_plane(3,keypoints_idx),10,'gh');
%         disp(strcat('Constructed heat map'));
%         axis equal;
%         pause         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%     
%     %%%%%%%Reset Variables%%%%%%
%     if (got_next==0)
%        frame_start=frame_start+50;
%        cam_frame_start=cam_frame_start+7;
%     else
%        frame_start=start_next_frame;
%        cam_frame_start=cam_start_next;
%     end
%     frame_end= frame_start+1;
%     frames=[frame_start];
%     i=frame_start;
%     j=i;               
%     got_next=0;    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end    
end






















