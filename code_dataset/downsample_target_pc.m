function output= downsample_target_pc(pointcloud, target_pc_size)
    %find appropriate scale
    scale_size=1.001;
    downsampled=pcdownsample(pointCloud(pointcloud'),'gridAverage',scale_size);

    while (downsampled.Count()<target_pc_size)
       scale_size=scale_size-0.025;
       if(scale_size<=0)
            xyz=pointcloud';
            break;
       end
       downsampled=pcdownsample(pointCloud(pointcloud'),'gridAverage',scale_size);
    end

    while (downsampled.Count()>target_pc_size)
       scale_size=scale_size+0.025;
       downsampled=pcdownsample(pointCloud(pointcloud'),'gridAverage',scale_size);
    end

    if(scale_size>0)
        xyz=[downsampled.Location(:,1),downsampled.Location(:,2),downsampled.Location(:,3)];
    end 

    %add additional random points
    num_extra_points=target_pc_size-size(xyz,1);
    permutation=randperm(length(pointcloud));
    sample_out=permutation(1:num_extra_points);
    sample=pointcloud(:,sample_out);%3xn            

    output=[xyz',sample];          
end