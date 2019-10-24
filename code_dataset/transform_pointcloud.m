function scaled_output =  transform_pointcloud(downsampled_pc)
    %transform wrt the centroid
    x_cen=mean(downsampled_pc(1,:));
    y_cen=mean(downsampled_pc(2,:));
    z_cen=mean(downsampled_pc(3,:));
%     centroid=[x_cen;y_cen;z_cen;1];
%     centroid_g=double(laser_global_poses{frame_start})*double(centroid);

    %make spread s=0.5/d
    sum=0;
    for i=1:size(downsampled_pc,2)
        sum=sum+sqrt((downsampled_pc(1,i)-x_cen)^2+(downsampled_pc(2,i)-y_cen)^2+(downsampled_pc(3,i)-z_cen)^2);
    end
    d=sum/size(downsampled_pc,2);
    s=0.5/d;

    T=[[s,0,0,-s*(x_cen)];...
    [0,s,0,-s*(y_cen)];...
    [0,0,s,-s*(z_cen)];...
    [0,0,0,1]];
    scaled_output=T*[downsampled_pc; ones(1, size(downsampled_pc,2))];
    scaled_output=-scaled_output;
end