% CAMSTRUCT2PROJMATRIX
%
% Usage: P = camstruct2projmatrix(C)
%
% Argument: C - Camera structure.
% Returns:  P - 3x4 camera projection matrix that maps homogeneous 3D world 
%               coordinates to homogeneous image coordinates.
%
% Function takes a camera structure and returns its equivalent projection matrix
% ignoring lens distortion parameters etc
%
% See also: CAMSTRUCT, PROJMATRIX2CAMSTRUCT, CAMERAPROJECT

% Copyright (c) Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% PK April 2015

function P = camstruct2projmatrix(C)
    
    K = [C.fx  C.skew  C.ppx  0
          0    C.fy    C.ppy  0
          0     0       1     0];
    
    T = [ C.Rc_w  -C.Rc_w*C.P
         0  0  0       1     ];
    
    P = K*T;