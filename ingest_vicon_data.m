function [vicon_trajectories, vicon_timestamps] = ingest_vicon_data(filename)

% load vicon data
vicon_data = readtable(filename);
vicon_objs = unique(vicon_data.object);
vicon_trajectories = cell(length(vicon_objs),1);

w = waitbar(0, '');
for i=1:size(vicon_data,1)
    ind = find(contains(vicon_objs, vicon_data{i,3}));
    vicon_trajectories{ind}{end+1,1} = reshape(vicon_data{i,11:end}',4,4)';
    waitbar(i/size(vicon_data,1),w, [num2str(i/size(vicon_data,1)) ' entries read']);
end
close(w);
vicon_timestamps = vicon_data{1:length(vicon_trajectories):end,1} + vicon_data{1:length(vicon_trajectories):end,2}/1e9;

end