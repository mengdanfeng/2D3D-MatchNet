%--------------------------------------------------------------------------
% Function to evaluate the first order approximation of the geometric error
% (Sampson distance) of the fit of a fundamental matrix with respect to a
% set of matched points as needed by RANSAC.  See: Hartley and Zisserman,
% 'Multiple View Geometry in Computer Vision', page 270.
%
% Note that this code allows for F being a cell array of fundamental matrices of
% which we have to pick the best one. (A 7 point solution can return up to 3
% solutions)

function [bestInliers, bestF] = funddist(F, x, t);
    
    x1 = x(1:3,:);    % Extract x1 and x2 from x
    x2 = x(4:6,:);
    
    
    if iscell(F)  % We have several solutions each of which must be tested
		  
	nF = length(F);   % Number of solutions to test
	bestF = F{1};     % Initial allocation of best solution
	ninliers = 0;     % Number of inliers
	
	for k = 1:nF
	    x2tFx1 = zeros(1,length(x1));
	    for n = 1:length(x1)
		x2tFx1(n) = x2(:,n)'*F{k}*x1(:,n);
	    end
	    
	    Fx1 = F{k}*x1;
	    Ftx2 = F{k}'*x2;     

	    % Evaluate distances
	    d =  x2tFx1.^2 ./ ...
		 (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
	    
	    inliers = find(abs(d) < t);     % Indices of inlying points
	    
	    if length(inliers) > ninliers   % Record best solution
		ninliers = length(inliers);
		bestF = F{k};
		bestInliers = inliers;
	    end
	end
    
    else     % We just have one solution
	x2tFx1 = zeros(1,length(x1));
	for n = 1:length(x1)
	    x2tFx1(n) = x2(:,n)'*F*x1(:,n);
	end
	
	Fx1 = F*x1;
	Ftx2 = F'*x2;     
	
	% Evaluate distances
	d =  x2tFx1.^2 ./ ...
	     (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
	
	bestInliers = find(abs(d) < t);     % Indices of inlying points
	bestF = F;                          % Copy F directly to bestF
	
    end