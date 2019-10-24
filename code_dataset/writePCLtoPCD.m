function writePCLtoPCD(pointcloud, pcd_path, pcd_fig_path)
% this function write pointcloud to PCD file
% pointcloud: 3xN
pcl = pointCloud(single(pointcloud'));
figure(1), pcshow(pcl),
hold on,
view(0,-90),
saveas(gcf,pcd_fig_path);
hold off
pcwrite(pcl, pcd_path, 'Encoding', 'ascii');
end