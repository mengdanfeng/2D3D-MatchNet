function rvec= RotationToEuler(R)
% this function convert totation matrix to euler angles
% input: 3x3 rotation matrix
% output: 1x3 rotation vector in degree
    x =atan2(R(3,2),R(3,3));
	y = atan2(-R(3,1), sqrt(R(3,2)*R(3,2) + R(3,3)*R(3,3)));
	z = atan2(R(2,1), R(1,1));
    
    rvec = [x,y,z]*180/pi;
end