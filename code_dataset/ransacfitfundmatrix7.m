% RANSACFITFUNDMATRIX7 - fits fundamental matrix using RANSAC
%
% Usage:   [F, inliers] = ransacfitfundmatrix7(x1, x2, t)
%
% This function requires Andrew Zisserman's 7 point fundamental matrix code. 
% See:  http://www.robots.ox.ac.uk/~vgg/hzbook/code/
%
% Arguments:
%          x1  - 2xN or 3xN set of homogeneous points.  If the data is
%                2xN it is assumed the homogeneous scale factor is 1.
%          x2  - 2xN or 3xN set of homogeneous points such that x1<->x2.
%          t   - The distance threshold between data point and the model
%                used to decide whether a point is an inlier or not. 
%                Note that point coordinates are normalised to that their
%                mean distance from the origin is sqrt(2).  The value of
%                t should be set relative to this, say in the range 
%                0.001 - 0.01  
%
% Note that it is assumed that the matching of x1 and x2 are putative and it
% is expected that a percentage of matches will be wrong.
%
% Returns:
%          F       - The 3x3 fundamental matrix such that x2'Fx1 = 0.
%          inliers - An array of indices of the elements of x1, x2 that were
%                    the inliers for the best model.
%
% See Also: RANSAC, FUNDMATRIX, RANSACFITFUNDMATRIX

% Copyright (c) 2004-2005 Peter Kovesi
% School of Computer Science & Software Engineering
% The University of Western Australia
% http://www.csse.uwa.edu.au/
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% February 2004  Original version
% August   2005  Distance error function changed to match changes in RANSAC
% June     2009  Bug in the wrapper function fixed (thanks to Peter Corke)

function [F, inliers] = ransacfitfundmatrix7(x1, x2, t, feedback)

    if ~all(size(x1)==size(x2))
        error('Data sets x1 and x2 must have the same dimension');
    end

    if nargin == 3
	feedback = 0;
    end
    
    [rows,npts] = size(x1);
    if rows~=2 & rows~=3
        error('x1 and x2 must have 2 or 3 rows');
    end
    
    if rows == 2    % Pad data with homogeneous scale factor of 1
        x1 = [x1; ones(1,npts)];
        x2 = [x2; ones(1,npts)];        
    end
    
    % Normalise each set of points so that the origin is at centroid and
    % mean distance from origin is sqrt(2).  normalise2dpts also ensures the
    % scale parameter is 1.  Note that 'fundmatrix' will also call
    % 'normalise2dpts' but the code in 'ransac' that calls the distance
    % function will not - so it is best that we normalise beforehand.
    [x1, T1] = normalise2dpts(x1);
    [x2, T2] = normalise2dpts(x2);

    s = 7;  % Number of points needed to fit a fundamental matrix using
            % a  7 point solution
    
    fittingfn = @vgg_F_from_7pts_wrapper;  % Wrapper for AZ's code
    distfn    = @funddist;
    degenfn   = @isdegenerate;
    % x1 and x2 are 'stacked' to create a 6xN array for ransac
    [F, inliers] = ransac([x1; x2], fittingfn, distfn, degenfn, s, t, feedback);

    % Now do a final least squares fit on the data points considered to
    % be inliers.
    F = fundmatrix(x1(:,inliers), x2(:,inliers));
    
    % Denormalise
    F = T2'*F*T1;

	 
    
