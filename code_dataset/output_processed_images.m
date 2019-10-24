base_path='/media/deep-three/Deep_Store/RobotCar_Images/extracted_data/2014-05-19-13-05-38/';
output_folder='/home/deep-three/Pictures/stereo_samples/';
camera_path='stereo/centre/';
timestamps=dlmread(strcat(base_path,'stereo.timestamps'));
ins_file=strcat(base_path,'gps/ins.csv');

camera_models_path='/media/deep-three/Deep_Store/RobotCar_Images/camera_models/';

%get LUT
[ ~, ~, ~, ~, ~, LUT] = ReadCameraModel(camera_path,camera_models_path);


% frame_start=1000;
% prev=frame_start;
% j=frame_start+1;
% 
counter=0;
% 
% pause;
% 
chunk=2;
cam_index_start=find(timestamps(:,2) == chunk, 1, 'first');
cam_index_end= find(timestamps(:,2) == chunk, 1, 'last');
timestamps=timestamps(cam_index_start:cam_index_end,1);
% 
% pause;
% poses=getGlobalPoses(ins_file, timestamps');

while(j<size(timestamps,1))
   while(counter<50)
%       if(counter==0)
%          I = LoadImage(strcat(base_path,camera_path), timestamps(counter,1), LUT);
%          imwrite(I,strcat(output_folder,num2str(timestamps(counter,1)),'.png'));
%          disp(num2str(counter));
%          counter=counter+1;
%       end 
      
%         if((getDistance(poses{prev}(1,4), poses{prev}(2,4),poses{j}(1,4), poses{j}(2,4))<1)...
%                        && (getRotation(poses{prev}(1:3,1:3),poses{j}(1:3,1:3))*180/pi <10))
%             j=j+1; 
%         else
         I = LoadImage(strcat(base_path,camera_path), timestamps(j,1), LUT);
         imwrite(I,strcat(output_folder,num2str(timestamps(j,1)),'.png'));
         disp(num2str(counter));
         counter=counter+1;
%          prev=j;
         j=j+1;
%         end   
        
   end 
end    