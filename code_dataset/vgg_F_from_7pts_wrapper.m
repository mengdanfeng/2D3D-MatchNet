%--------------------------------------------------------------------------
% Function providing a wrapper for Andrew Zisserman's 7 point fundamental
% matrix code. See:  http://www.robots.ox.ac.uk/~vgg/hzbook/code/
% This code takes inputs and returns output according to the requirements of
% RANSAC
    
function F = vgg_F_from_7pts_wrapper(x)
    
    Fvgg = vgg_F_from_7pts_2img(x(1:3,:), x(4:6,:));

    if isempty(Fvgg)
	F = [];
	return;
    end
    
    % Store the (potentially) 3 solutions in a cell array
    [rows,cols,Nsolutions] = size(Fvgg);
    for n = 1:Nsolutions
	F{n} = Fvgg(:,:,n);
    end