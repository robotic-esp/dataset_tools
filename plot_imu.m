function [] = plot_imu(lin_acc, quaternions)
%
% Plots the linear acceleration vector, and rotation quaternion as recorded
% by the imu.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   lin_acc: a Kx3 matrix of the linear acceleration vector over time
%   quaternions: a Kx4 matrix of the rotation quaternion over time
%
% output:
%

if(size(lin_acc, 1) ~= size(quaternions, 1))
    error('ERROR - input length mismatched');
end

neg_bound = @(x) min(0, max(-inf, x));
pos_bound = @(x) min(inf, max(0, x));

axis_lims = [neg_bound(min(lin_acc(:,1))) pos_bound(max(lin_acc(:,1))) neg_bound(min(lin_acc(:,2))) ...
             pos_bound(max(lin_acc(:,2))) neg_bound(min(lin_acc(:,3))) pos_bound(max(lin_acc(:,3)))];

for i = 1:10:size(lin_acc,1)
    subplot(1,2,1);
    quiver3(0, 0, 0, lin_acc(i,1), lin_acc(i,2), lin_acc(i,3),'LineWidth',5,'MaxHeadSize',1)
    grid on;
    axis(axis_lims);
    drawnow;
    
    subplot(1,2,2);cla;
    R = quaternion_to_rotation_matrix(quaternions(i,:));
    T = [R [0; 0; 0;]; 0 0 0 1];
    plot_axes3(T,0.25);
    grid on;
    axis(0.5*[-1 1 -1 1 -1 1]);
    drawnow;
end

end

function [R] = quaternion_to_rotation_matrix(q)
x = q(1); y = q(2); z = q(3); w = q(4);
Nq = w*w + x*x + y*y + z*z;
s = 2.0/Nq;
X = x*s;
Y = y*s;
Z = z*s;
wX = w*X; wY = w*Y; wZ = w*Z;
xX = x*X; xY = x*Y; xZ = x*Z;
yY = y*Y; yZ = y*Z; zZ = z*Z;
R =  [1.0-(yY+zZ), xY-wZ, xZ+wY;
      xY+wZ, 1.0-(xX+zZ), yZ-wX;
      xZ-wY, yZ+wX, 1.0-(xX+yY)];
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
    axes_in_i = scale * [0 1 0 0;
                 0 0 1 0;
                 0 0 0 1;
                 1/scale 1/scale 1/scale 1/scale];
    if(ischar(colors))
     colors = colors';
    end
    axes_in_j = (T_j_from_i*axes_in_i);
    holdstate = ishold;
    hold on
    plot3([axes_in_j(1,1) axes_in_j(1,2)], [axes_in_j(2,1) axes_in_j(2,2)], ...
             [axes_in_j(3,1) axes_in_j(3,2)], 'Color', colors(1,:), ...
             'LineWidth', thickness);
    plot3([axes_in_j(1,1) axes_in_j(1,3)], [axes_in_j(2,1) axes_in_j(2,3)], ...
             [axes_in_j(3,1) axes_in_j(3,3)], 'Color', colors(2,:), ...
             'LineWidth', thickness);
    plot3([axes_in_j(1,1) axes_in_j(1,4)], [axes_in_j(2,1) axes_in_j(2,4)], ...
             [axes_in_j(3,1) axes_in_j(3,4)], 'Color', colors(3,:), ...
             'LineWidth', thickness);
    if ~holdstate
        hold off
    end
end