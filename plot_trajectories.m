function [] = plot_trajectories(trajectories, f, plotrange, stepsize)
%
% Incrementally plots the motion histories defined by trajectories.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   trajectories: Mx1 cell array of the M objects tracked in the
%   vicon file, where each cell contains a Kx1 cell array of global 4x4
%   transformation matrices representing each trajectory
%   f: figure handle on which to plot the trajectories
%   plotrange: 2x1 array defining the lower and upper index limits to plot
%   stepsize: increment between plot steps
%
% output:
%

if nargin == 1
    f = figure;
    plotrange = [1 length(trajectories{1})];
    stepsize = 1;
elseif nargin == 2
    plotrange = [1 length(trajectories{1})];
    stepsize = 1;
elseif nargin == 3
    stepsize = 1;
end

for i = 2:length(trajectories)
    if length(trajectories{i}) < plotrange(2)
        error('ERROR - mismatched trajectory lengths or plotrange limit too large')
    end
end

set(0, 'CurrentFigure', f)
hold off;
for j = 1:length(trajectories)
    plot_trajectory(trajectories{j}(plotrange(1):stepsize:plotrange(2)));
    hold on
end

end

