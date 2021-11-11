function [max_xyz_error, max_xyz_percent_error, max_rpy_error] = compare_estimate_to_ground_truth(trajectory_estimate, trajectory_gt, T_estimate_from_gt)
%   
% Compares estimated trajectory to the ground-truth trajectory and plots the xyz
% and roll-pitch-yaw error in the ground-truth frame. The estimated trajectory
% is calibrated to the ground-truth frame via the T_estimate_from_gt transform.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   trajectory_estimate: a Kx1 cell array of 4x4 transformation matrices
%   representing a estimated motion trajectory
%   trajectory_gt: a Kx1 cell array of 4x4 transformation matrices
%   representing the gorund-truth motion trajectory
%   T_estimate_from_gt: a 4x4 transformation matrix representing the calibration
%   from the ground-truth frame to the estimated frame
%
% output:
%   max_xyz_error: the xyz error at the maximally displaced point along the trajectory
%   max_xyz_percent_error: the max_xyz_error as a percentage of the total path at that point
%   max_rpy_error: the rpy error at the maximally displaced point along the trajectory
%

if(length(trajectory_estimate) ~= length(trajectory_gt))
    error('Mismatched lengths');
end

xyz_err = zeros(length(trajectory_estimate),3);
rpy_err = zeros(length(trajectory_estimate),3);
for i = 1:length(trajectory_estimate)

    T_err = invT(T_estimate_from_gt) * trajectory_estimate{i} * T_estimate_from_gt * invT(trajectory_gt{i});
    
    tmp = get_r_j_from_i_in_i_FROM_T_ji(T_err);
	xyz_err(i,:) = tmp(1:3);
    rpy_err(i,:) = rotmat_to_rpy(T_err(1:3,1:3))';
end

path = total_path_length(trajectory_gt);

[max_xyz_error, i] = max(sqrt(sum(xyz_err.^2,2)));
max_xyz_percent_error = max_xyz_error / path(i);
[~, i] = max(sqrt(sum(rpy_err.^2,2)));
max_rpy_error = rpy_err(i,:)*180/pi;
avg_xyz_error = mean(sqrt(sum(xyz_err.^2,2)));
avg_rpy_error = mean(sqrt(sum(rpy_err.^2,2)));

figure;
subplot(2,1,1);
hold on
plot(path, xyz_err)
set(gca, 'Box', 'off');
ylabel('Tran. Error (m)')
legend({'x','y','z'}, 'Location', 'Southwest', 'Orientation','horizontal');
box off
xticklabels([])
xlim([0, path(end)]);


subplot(2,1,2);
hold on
plot(path, rpy_err)
ylabel('Rot. Error (rad)')
ylimits_rad = ylim;
ylim(ylimits_rad);
hold on
plot(path, rpy_err)
yyaxis right
ax = gca;
ax.ColorOrderIndex = 1;
ax.YColor = 'k';
ylim(ylimits_rad./pi.*180)
ylabel('Rot. Error (deg)')
legend({'roll','pitch','yaw'}, 'Location', 'Northwest', 'Orientation','horizontal');
box off
if(path_label)
    xlabel('Path Length (m)')
else
    xlabel('Frames')
end
xlim([0, path(end)]);

linkaxes([subplot(2,1,1),subplot(2,1,2)], 'x');

end

function [ path_length ] = total_path_length( trajectory )
%TOTAL_PATH_LENGTH Total path length of a trajectory of points

N = length(trajectory);
trajectory_points = zeros(4,N);
for i = 1:N
    trajectory_points(:,i) = invT(trajectory{i}) * [0 0 0 1]';
end
path_length = zeros(N,1);

for i=2:N
    path_length(i) = path_length(i-1) + sqrt(sum((trajectory_points(:,i) - trajectory_points(:,i-1)).^2));
end

end


function [rpy] = rotmat_to_rpy(C)
    if ~ismatrix(C) || size(C,1) ~= 3 || size(C,2) ~= 3
        error('C must be 3x3.')
    end
    
    rpy(3,1) = atan2(C(2,1),C(1,1));
    rpy(2,1) = atan2(-C(3,1),sqrt(C(3,2)^2 + C(3,3)^2));
    rpy(1,1) = atan2(C(3,2),C(3,3));
end
