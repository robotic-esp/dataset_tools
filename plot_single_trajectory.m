function [] = plot_single_trajectory(trajectory)
%
% Plots the motion history defined by trajectory, including 3-D axes at the
% final position and orientation of the trajectory.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   trajectory: a Kx1 cell array of global 4x4 transformation 
%   matrices representing a discrete motion trajectory
%
% output:
%

pts = zeros(4,length(trajectory));
for i = 1:length(trajectory)
    pts(:,i) = get_r_j_from_i_in_i_FROM_T_ji(invT(trajectory{i}));
end

plot3(pts(1,1:i),pts(2,1:i),pts(3,1:i))
plot_axes3(trajectory{i},0.25);

end

function [r_j_from_i_in_i] = get_r_j_from_i_in_i_FROM_T_ji(T_ji)
% Extracts the translation vector from the origin frame, F_i, to the
% target frame, F_j, expressed in the origin frame, F_i. 

    r_i_from_j_in_j = T_ji(1:3,4);
    
    C_ji = T_ji(1:3,1:3);
    r_j_from_i_in_i = -C_ji' * r_i_from_j_in_j;
    r_j_from_i_in_i = [r_j_from_i_in_i;1];
end

function [] = plot_axes3(T_j_from_i, varargin)
% Draws 3D axes at the position and orientation given by T_j_from_i
%
% input:
%   T_j_from_i: position and orientation of the axes to be drawn
%   optional arguments:
%       colors: 3-character string or 3x3 matrix defining the colors of the
%       x-, y-, and z-axes respectively
%       scale: length of the axis lines
%       thickness: thickness of the axis lines
%
% output:
%
    if nargin < 1
        return
    elseif nargin == 1
        colors = 'brg';
        scale = 2;
        thickness = 2;
    elseif nargin == 2
        scale = varargin{1};
        colors = 'brg';
        thickness = 2; 
    elseif nargin == 3
        scale = varargin{1};
        colors = varargin{2};
        thickness = 2; 
    else
        scale = varargin{1};
        colors = varargin{2};
        thickness = varargin{3};
    end
    axes_in_j = scale * [0 1 0 0;
                 0 0 1 0;
                 0 0 0 1;
                 1/scale 1/scale 1/scale 1/scale];
    if(ischar(colors))
     colors = colors';
    end
    axes_in_i = (invT(T_j_from_i)*axes_in_j);
    holdstate = ishold;
    hold on
    plot3([axes_in_i(1,1) axes_in_i(1,2)], [axes_in_i(2,1) axes_in_i(2,2)], ...
             [axes_in_i(3,1) axes_in_i(3,2)], 'Color', colors(1,:), ...
             'LineWidth', thickness);
    plot3([axes_in_i(1,1) axes_in_i(1,3)], [axes_in_i(2,1) axes_in_i(2,3)], ...
             [axes_in_i(3,1) axes_in_i(3,3)], 'Color', colors(2,:), ...
             'LineWidth', thickness);
    plot3([axes_in_i(1,1) axes_in_i(1,4)], [axes_in_i(2,1) axes_in_i(2,4)], ...
             [axes_in_i(3,1) axes_in_i(3,4)], 'Color', colors(3,:), ...
             'LineWidth', thickness);
    if ~holdstate
        hold off
    end
end