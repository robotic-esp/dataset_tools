function [vicon_trajectories, vicon_objects, vicon_timestamps] = ingest_vicon_data(filename)
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

w = waitbar(0, 'Opening file');

% load vicon data
vicon_data = readtable(filename);
vicon_objects = unique(vicon_data.object);
vicon_trajectories = cell(length(vicon_objects),1);

w = waitbar(0, w, ['0/' num2str(size(vicon_data,1)) ' entries read']);

% preallocate
for i=1:length(vicon_trajectories)
    vicon_trajectories{i} = cell(size(vicon_data,1)/length(vicon_trajectories),1);
    [vicon_trajectories{i}{:,1}] = deal(zeros(4,4)); % See: help deal
end
write_idx = zeros(length(vicon_trajectories),1);

for i=1:size(vicon_data,1)
    ind = find(contains(vicon_objects, vicon_data{i,3}));
    write_idx(ind) = write_idx(ind) + 1;
    vicon_trajectories{ind}{write_idx(ind),1} = reshape(vicon_data{i,11:end}',4,4)';
    if mod(i,100) == 0 || i == size(vicon_data,1)
        waitbar(i/size(vicon_data,1), w, [num2str(i) '/' num2str(size(vicon_data,1)) ' entries read']);
    end
end
close(w);
vicon_timestamps = vicon_data{1:length(vicon_trajectories):end,1} + vicon_data{1:length(vicon_trajectories):end,2}/1e9;

end
