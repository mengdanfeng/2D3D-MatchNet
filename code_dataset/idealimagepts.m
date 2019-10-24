% IDEALIMAGEPTS - Ideal image points with no distortion.
%
% Usage:  xyideal = idealimagepts(C, xy)
%
% Arguments:  
%          C - Camera structure, see CAMSTRUCT for definition.  
%         xy - Image points specified as 2 x N array (x,y) / (col,row)
%
% Returns:
%    xyideal - Ideal image points.  These points correspond to the image
%              locations that would be obtained if the camera had no lens
%              distortion.  That is, if they had been projected using an ideal
%              projection matrix computed from fx, fy, ppx, ppy, skew.
%
% See also CAMSTRUCT, CAMERAPROJECT, SOLVESTEREOPT

% Copyright (c) 2015 Peter Kovesi
% pk at peterkovesi com
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% August   2015 Adapted from imagept2plane
% February 2016 Refined inversion of distortion process slightly

function xyideal = idealimagepts(C, xy)
    
    [dim,N] = size(xy);
    if dim ~= 2
        error('Image xy data must be a 2 x N array');
    end
    
    % Reverse the projection process as used by CAMERAPROJECT

    % Subtract principal point and divide by focal length to get normalised,
    % distorted image coordinates.  Note skew represents the 2D shearing
    % coefficient times fx
    y_d = (xy(2,:) - C.ppy)/C.fy;   
    x_d = (xy(1,:) - C.ppx - y_d*C.skew)/C.fx;   
    
    % Compute the inverse of the lens distortion effect by computing the
    % 'forward' direction lens distortion at this point and then subtracting
    % this from the current point.  

    % Radial distortion factor. Here the squared radius is computed from the
    % already distorted coordinates.  The approximation we are making here is to
    % assume that the distortion is locally constant.
    rsqrd = x_d.^2 + y_d.^2;
    r_d = 1 + C.k1*rsqrd + C.k2*rsqrd.^2 + C.k3*rsqrd.^3;
    
    % Tangential distortion component, again computed from the already distorted
    % coords.
    dtx = 2*C.p1*x_d.*y_d         + C.p2*(rsqrd + 2*x_d.^2);
    dty = C.p1*(rsqrd + 2*y_d.^2) + 2*C.p2*x_d.*y_d;

    % Subtract the tangential distortion components and divide by the radial
    % distortion factor to get an approximation of the undistorted normalised
    % image coordinates (with no skew)
    x_n = (x_d - dtx)./r_d;
    y_n = (y_d - dty)./r_d;
    
    % Finally project back to pixel coordinates.
    x_p = C.ppx + x_n*C.fx + y_n*C.skew;
    y_p = C.ppy + y_n*C.fy;    
    
    xyideal = [x_p
               y_p];

