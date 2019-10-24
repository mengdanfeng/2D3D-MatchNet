%file = '/media/mengdan/data3/robotcar/grasshopper/iss_volume/2014-07-14-15-16-36/028/cam1_0069_0004_00010.pcd';
%file = '/media/mengdan/data3/robotcar/grasshopper/iss_volume/2014-07-14-15-16-36/036/cam1_0095_0001_00026.pcd';
%file = '/media/mengdan/data3/robotcar/grasshopper/iss_volume/2014-07-14-15-16-36/064/cam1_0073_0013_00020.pcd';
%file = '/media/mengdan/data3/robotcar/grasshopper/iss_volume/2014-07-14-15-16-36/064/cam2_0186_0070_00065.pcd';
file = '/media/mengdan/data3/robotcar/grasshopper/iss_volume/2014-07-14-15-16-36/064/cam2_0186_0073_00068.pcd';
%file='/media/mengdan/data3/robotcar/grasshopper/iss_volume/2014-07-14-15-16-36/048/cam1_0396_0023_00077.pcd';
pcl = pcread(file);
xyz = pcl.Location;     % nx3
color = xyz(:,2);
pcshow(xyz, color, 'MarkerSize', 28)
set(gcf,'color','w');
hold on
axis off
grid off
