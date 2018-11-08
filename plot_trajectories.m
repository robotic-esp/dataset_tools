function [f] = plot_trajectories(trajectories, f, plotrange, stepsize)


% for i = 1:100:length(trajs{1})
% plot_trajectories(trajs, f, [1 i]);
% view(-20, 30)
% axis([-2 1.5 -6 2 0.5 2.5])
% drawnow
% end

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

