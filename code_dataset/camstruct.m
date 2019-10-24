% CAMSTRUCT - Construct a camera structure 
%
% Usage: C = camstruct(param_name, value, ...
%
% Where the parameter names can be as follows with defaults in brackets
%
%           fx - X focal length in pixel units. ([])
%           fy - Y focal length in pixel units. ([])
%  or        f - Focal length in pixel units, the same value is copied to
%                both fx and fy.  ([])
%     ppx, ppy - Principal point.                       (0,0)
%   k1, k2, k3 - Radial lens distortion parameters.     (0,0,0)
%       p1, p2 - Tangential lens distortion parameters. (0,0)
%         skew - Camera skew. (0)
%   rows, cols - Image size.  (0,0)
%            P - 3D location of camera in world coordinates. ([0;0;0])
%         Rc_w - 3x3 rotation matrix describing the world relative to the
%                camera  frame.   (eye(3))
%
%   Alternatively P and Rc_w can be replaced with:
%         Tw_c - 4x4 homogeneous transformation matrix describing camera
%                position and orientation with respect to world
%                coordinates. ([])
%
% Example:
%   >> C = camstruct('f', 2000, 'k1', 1.5, 'P', [0;0;2000], 'Rc_w', rotx(pi));
%
%   Alternatively you can construct a camera structure from a projection
%   matrix using the parameter name 'PM'
%
%   >> C = camstruct('PM', proj_matrix);
%
% Returns:   C - Camera structure with fields: 
%                fx, fy, ppx, ppy, k1, k2, k3, k4, skew, rows, cols, P, Rc_w
%
% Note rows and cols, if supplied, are only used to determine if projected
% points fall within the image bounds by CAMERAPROJECT.
%
% Note the ordering of the tangential distortion parameters p1 and p2 is not
% always consistent in the literature.  Within CAMERAPROJECT they are used in
% the following order.
%    dx = 2*p1*x*y          +  p2*(r^2 + 2*x^2)
%    dy = p1*(r^2 + 2*y^2)  +  2*p2*x*y
%
% See also: CAMERAPROJECT, CAMSTRUCT2PROJMATRIX, IMAGEPT2PLANE

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

% April   2015
% August  2015   Change to 3 radial distortion parameters k1, k2, k3 and two
%                tangential parameters p1, p2
% January 2016   Allow for Rc_w to be supplied as a 4x4 homogeneous matrix

function C = camstruct(varargin)

% !! This code is awful, I wish MATLAB was a proper language which allowed you
% to specify optional argument values in the function definition and also had
% proper constructors for structures/objects, and had multiple dispatch !!

% I am moving all my code over to Julia...    

    % Parse the input arguments and set defaults
    p = inputParser;
    p.CaseSensitive = false;
    
    % Define optional parameter-value pairs and their defaults    
    addParameter(p, 'PM',    [], @isnumeric);  
    addParameter(p, 'f',     [], @isnumeric);  
    addParameter(p, 'fx',    [], @isnumeric);  
    addParameter(p, 'fy',    [], @isnumeric);  
    addParameter(p, 'ppx',    0, @isnumeric);  
    addParameter(p, 'ppy',    0, @isnumeric);  
    addParameter(p, 'k1',     0, @isnumeric);  
    addParameter(p, 'k2',     0, @isnumeric);  
    addParameter(p, 'k3',     0, @isnumeric);  
    addParameter(p, 'p1',     0, @isnumeric);  
    addParameter(p, 'p2',     0, @isnumeric);
    addParameter(p, 'skew',   0, @isnumeric);  
    addParameter(p, 'rows',   0, @isnumeric);  
    addParameter(p, 'cols',   0, @isnumeric);     
    addParameter(p, 'P',   [0;0;0], @isnumeric);  
    addParameter(p, 'Rc_w', eye(3), @isnumeric);  
    addParameter(p, 'Tw_c',     [], @isnumeric);  
    
    parse(p, varargin{:});
    
    PM  = p.Results.PM;    
    f   = p.Results.f;
    
    fx  = p.Results.fx;
    fy  = p.Results.fy;
    ppx = p.Results.ppx;
    ppy = p.Results.ppy;
    k1  = p.Results.k1;
    k2  = p.Results.k2;
    k3  = p.Results.k3;
    p1  = p.Results.p1;
    p2  = p.Results.p2;
    skew = p.Results.skew;
    rows  = p.Results.rows;
    cols  = p.Results.cols;
    P    = p.Results.P(:);
    Rc_w = p.Results.Rc_w;
    Tw_c = p.Results.Tw_c;
    
    % Handle case when projection matrix is supplied
    if ~isempty(PM)
        if ~all(size(PM) == [3,4])
            error('Projection matrix must be 3x4');
        end
        
        C = projmatrix2camstruct(PM);
        
        % It is possible, though unlikely, the user wanted to specify some lens
        % distortion parameters in addition to the projection matrix.
        C.k1 = k1;
        C.k2 = k2;
        C.k3 = k3;
        C.p1 = p1;
        C.p2 = p2;
        
        return
    end    
    
    % Handle case where f is supplied, copy it to fx, fy
    if ~isempty(f) && isempty(fx) && isempty(fy)
        fx = f;
        fy = f;
    elseif ~isempty(f) && (~isempty(fx) || ~isempty(fy))
        error('Cannot specify f and fx or fy');
    elseif isempty(fx) || isempty(fy)
        error('Focal length not defined');
    end
        
    % Handle case when Tw_c is supplied
    if ~isempty(Tw_c)
        P = Tw_c(1:3,4);
        Rc_w = Tw_c(1:3,1:3)';
    elseif ~isempty(Tw_c) && (~isempty(P) || ~isempty(Rc_w))
        error('Cannot specify Tw_c and P or Rc_w');
    else
        % Rc_w has been supplied.  If it has been supplied as a 4x4
        % homogeneous transform just extract the 3x3 rotation component
        if all(size(Rc_w) == [4,4])
            Rc_w = Rc_w(1:3,1:3)
        end
    end    
    
    if ~all(size(P) == [3,1])
        error('Camera position must be 3x1 vector');
    end    
    
    if ~all(size(Rc_w) == [3,3])    
        error('Camera pose must be specified via a 3x3 rotation matrix or 4x4 homogeneous matrix');
    end
    
    C = struct('fx', fx, 'fy', fy, ...
               'ppx', ppx, 'ppy', ppy, ...
               'k1', k1, 'k2', k2, 'k3', k3, ...
               'p1', p1, 'p2', p2, ...
               'skew', skew, ...
               'rows', rows, 'cols', cols, ...
               'P', P(:), 'Rc_w', Rc_w);
    
    