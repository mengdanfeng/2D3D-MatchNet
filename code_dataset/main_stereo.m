close all
clear all
clc

imageDate = '2014-12-16-18-44-24';
% imageDate = '2015-02-03-08-45-10';
dirPath = '/media/deep-two/My Passport/RoboCarData/';
cameraName = 'centre';
imagePrefix = strcat(dirPath, imageDate, '_stereo_', cameraName, '_0');
imageSuffix = strcat('/', imageDate, '/stereo/', cameraName, '/');
timeStampPath = strcat(dirPath, imageDate, '_stereo_', cameraName, '.timestamps'); 

% get timestamps
timeStampsTemp = load(timeStampPath);

% get number of folders
nFolders = length(unique(timeStampsTemp(:,2)));

for folderID=1:nFolders
    idx = find(timeStampsTemp(:,2) == folderID);
    timeStamps{folderID} = timeStampsTemp(idx,1);
end

closest=999999999999;
base_timestamp=1418757326971333;
found_timestamp=0;
folder=-1;

for j=1:size(timeStamps,2)
   set_timestamps=timeStamps{j};
   for k=1:size(set_timestamps,1)
       current=abs(base_timestamp-set_timestamps(k));
       if(current<closest )
           closest=current;
           found_timestamp=set_timestamps(k);
           folder=j;
%            break;
       end    
   end
end
disp(strcat( imagePrefix, sprintf('%d', folder), imageSuffix, strcat(num2str(found_timestamp),'.png') ));
imgFileName = strcat( imagePrefix, sprintf('%d', folder), imageSuffix, strcat(num2str(found_timestamp),'.png') );

imgRGB = demosaic(imread(imgFileName), 'gbrg');
% im2=imresize(imgRGB,0.5);
im2=imgRGB;
g=figure;
imshow(im2);


% folderID=1;
% for imgCnt=1000:1:size(timeStamps{folderID})
%     
%     imgFileName = strcat( imagePrefix, sprintf('%d', folderID), imageSuffix, sprintf('%d.png', timeStamps{folderID}(imgCnt)) );
%     disp(strcat( imagePrefix, sprintf('%d', folderID), imageSuffix, sprintf('%d.png', timeStamps{folderID}(imgCnt)) ));
%    
%     imgRGB = demosaic(imread(imgFileName), 'gbrg');
%     imgGray = single(rgb2gray(imgRGB));
% %     [f,d] = vl_sift(imgGray, 'PeakThresh', 2);
%     im2=imresize(imgRGB,0.5);
% %     imshow(imgRGB)
%     imshow(im2);
% %     hold on
% %     plot(f(1,:),f(2,:),'g.', 'markersize', 10)
% %     hold off
% 
%     pause
%     
% end