function [vicon_trajectories, vicon_timestamps] = ingest_vicon_data(filename)
%
% Parses vicon data from input csv into trajectory and timestamp data.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   filename: input csv filename
%
% output:
%   vicon_trajectories: Mx1 cell array of the M objects tracked in the
%   vicon file, where each cell contains a Kx1 cell array of global 4x4
%   transformation matrices representing each trajectory
%   vicon_timestamps: Kx1 vector of timestamps
%

% load vicon data
vicon_data = readtable(filename);
vicon_objs = unique(vicon_data.object);
vicon_trajectories = cell(length(vicon_objs),1);

w = waitbar(0, '');
for i=1:size(vicon_data,1)
    ind = find(contains(vicon_objs, vicon_data{i,3}));
    vicon_trajectories{ind}{end+1,1} = reshape(vicon_data{i,11:end}',4,4)';
    waitbar(i/size(vicon_data,1),w, [num2str(i) '/' num2str(size(vicon_data,1)) ' entries read']);
end
close(w);
vicon_timestamps = vicon_data{1:length(vicon_trajectories):end,1} + vicon_data{1:length(vicon_trajectories):end,2}/1e9;

end