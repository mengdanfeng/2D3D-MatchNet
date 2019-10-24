% CAMERAPROJECT - Projects 3D points into camera image 
%
% Usage:  [xy, visible] = cameraproject(C, pt)    
%
% Arguments:
%              C - Camera structure, see CAMSTRUCT for definition.
%                  Alternatively C can be a 3x4 camera projection matrix.
%             pt - 3xN matrix of 3D points to project into the image.
%
% Returns: 
%              xy      - 2xN matrix of projected image positions
%              visible - Array of values 1/0 indicating whether the point is
%                        within the field of view.  This is only evaluated if
%                        the camera structure has non zero values for its
%                        'rows' and 'cols' fields. Otherwise an empty matrix
%                        is returned, an empty matrix is also returned if C
%                        is a 3x4 projection matrix.
%
% Note the ordering of the tangential distortion parameters p1 and p2 is not
% always consistent in the literature.  Here they are used in the following
% order.
%    dx = 2*p1*x*y          +  p2*(r^2 + 2*x^2)
%    dy = p1*(r^2 + 2*y^2)  +  2*p2*x*y
%
% See also: CAMSTRUCT, CAMSTRUCT2PROJMATRIX, IMAGEPT2PLANE

% Copyright (c) 2008-2015 Peter Kovesi
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

% PK September 2008
%    April     2015    - Refactored from CAMERAPROJECTV for camera structure
%                        which now includes fx, fy.
%          May 2015    - Simplified by removing R and rotscale and, Tc_w reduced
%                        to a rotation matrix Rc_w, Allow for C to be a
%                        projection matrix as well.
%      August 2015     - Changes to accommodate k1,k2,k3 radial parameter
%                        and p1,p2 tangential parameter renaming.
%    December 2015     - Bug fix for k3 when pt represents multiple points.

function [xy, visible] = cameraproject(C, pt)

    [rows, npts] = size(pt);
    
    if rows ~= 3
        error('Points must be in a 3xN array');
    end
    
    if ~isstruct(C) && all(size(C) == [3,4])  % C is a projection matrix
        xy = C*makehomogeneous(pt);
        xy = makeinhomogeneous(xy);
        visible = [];
        return;
    end
    
    % If we get here C is a camera structure and we follow the classical
    % projection process.
    
    % If only one focal length specified in structure use it for both fx and fy
    if isfield(C, 'f')
        fx = C.f;
        fy = C.f;
    elseif isfield(C, 'fx') && isfield(C, 'fy')
        fx = C.fx;
        fy = C.fy;        
    else
        error('Invalid focal length specification in camera structure');
    end    
    
    if isfield(C, 'skew')     % Handle optional skew specfication
        skew = C.skew;
    else
        skew = 0;
    end
    
    % Transformation of a ground point from world coordinates to camera coords
    % can be thought of as a translation, then rotation as follows
    %
    %   Gc =  Tc_p *  Tp_w * Gw    
    %
    %  | . . .   | | 1     -x | |Gx|
    %  | Rc_w    | |   1   -y | |Gy|
    %  | . . .   | |     1 -z | |Gz|
    %  |       1 | |        1 | | 1|
    %      
    % Subscripts:
    %   w - world frame
    %   p - world frame translated to camera origin
    %   c - camera frame

    % First translate world frame origin to match camera origin.  This is
    % the Tp_w bit.  
    for n = 1:3
        pt(n,:) = pt(n,:) - C.P(n); 
    end
    
    % Then rotate to camera frame using Rc_w
    pt = C.Rc_w*pt;
    
    % Follow Bouget's projection process    
    % Generate normalized coords
    x_n = pt(1,:)./pt(3,:);
    y_n = pt(2,:)./pt(3,:);
    rsqrd = x_n.^2 + y_n.^2;  % radius squared from centre
    
    % Radial distortion factor
    r_d = 1 + C.k1*rsqrd + C.k2*rsqrd.^2 + C.k3*rsqrd.^3;
    
    % Tangential distortion component  
    dtx = 2*C.p1*x_n.*y_n         + C.p2*(rsqrd + 2*x_n.^2);
    dty = C.p1*(rsqrd + 2*y_n.^2) + 2*C.p2*x_n.*y_n;
    
    % Obtain lens distorted coordinates
    x_d = r_d.*x_n + dtx;
    y_d = r_d.*y_n + dty;   
    
    % Finally project to pixel coordinates
    % Note skew represents the 2D shearing coefficient times fx
    x_p = C.ppx + x_d*fx + y_d*skew;
    y_p = C.ppy + y_d*fy;    
    
    xy = [x_p
	  y_p];
    
    % If C.rows and C.cols ~= 0 determine points that are within image bounds
    if C.rows && C.cols
        visible = x_p >= 1 & x_p <= C.cols & y_p >= 1 & y_p <= C.rows;
    else
        visible = [];
    end
