% SOLVESTEREOPT - Homogeneous linear solution of a stereo point
%
% Usage:  [pt, xy_reproj] = solvestereopt(xy, P)
%
% Multiview stereo: Solves 3D location of a point given image coordinates of
% that point in two, or more, images.
%
% Arguments:    xy - 2xN matrix of x, y image coordinates, one column for
%                    each camera.
%                C - N element cell array of corresponding image projection
%                    matrices. Or an N element cell array of camera structures
%
% Returns:      pt - 3D location in space returned in normalised
%                    homogeneous coordinates (a 4-vector with last element = 1)
%        xy_reproj - 2xN matrix of reprojected image coordinates.
%
%
% See also: CORRECTIMAGEPTS, CAMSTRUCT2PROJMATRIX

% Copyright (c) 2011-2016 Peter Kovesi
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

% May      2011
% February 2016   Provision for either camera projection matrices or camera
%                 structs 

function [pt, xy_reproj] = solvestereopt(xy, C)

    [dim,N] = size(xy);
    assert(N == length(C));
    assert(dim == 2);
    assert(N >= 2);
    
    % Determine if C is a cell array of projection matrices or camera
    % structures
    if isstruct(C{1})  % Assume camera structure
        % Correct image points from each camera so that they correspond to image
        % points from an ideal camera with no lens distortion.  Also generate the
        % corresponding ideal projection matrices for each camera struct
        for n = 1:N
            xy(:,n) = idealimagepts(C{n}, xy(:,n)); 
            P{n} = camstruct2projmatrix(C{n});
        end
        
    elseif isnumeric(C{1}) % Assume projection matrix
        P = C;
    else
        error('Camera argument must be a camera structure or projection matrix');
    end
    
    % Build eqn of the form A*pt = 0
    A = zeros(2*N, 4);
    for n = 1:N
	A(2*n-1,:) = xy(1,n)*P{n}(3,:) - P{n}(1,:);
	A(2*n  ,:) = xy(2,n)*P{n}(3,:) - P{n}(2,:);
    end
	
    [~,~,v] = svd(A);
    pt = hnormalise(v(:,4));
    
    if nargout == 2
        % Project the point back into the source images to determine the residual
        % error
        xy_reproj = zeros(size(xy));
        for n = 1:N
            xy_reproj(:,n) = cameraproject(P{n}, pt(1:3));
        end    
    end
    