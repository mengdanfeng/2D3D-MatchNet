function distance= getDistance1(pose1, pose2)
    
    x1 = pose1(1,4);
    y1 = pose1(2,4);
    x2 = pose2(1,4);
    y2 = pose2(2,4);
    
    distance= sqrt((x2-x1)^2+(y2-y1)^2);
end
