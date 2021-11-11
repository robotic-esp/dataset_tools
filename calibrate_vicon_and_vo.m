%
% Calibrates an estimated VO trajectory (provided by the user in vo_data)
% to the ground-truth trajectory.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk


path_to_vicon_data = ''; % path to vicon.csv file
path_to_images = ''; % path to the directory containing the stereo RGB images


% load vo data
vo_data = {eye(4)}; % load in VO estimates


start_index = 100; % starting frame index of the estimated data
end_index = start_index + 923; % final frame index of the estimated data
timestamps = readtable([path_to_images '/stereo.csv']);
vo_timestamps = timestamps{start_index:end_index,2} + timestamps{start_index:end_index,3}/1e9; 

% load vicon data
[vicon_trajectories, vicon_objects, vicon_timestamps] = ingest_vicon_data(path_to_vicon_data);

% plot vicon data
plot_multiple_trajectories(vicon_trajectories)

% find vicon transforms corresponding to VO timestamps
start_vicon = 1;
while(vo_timestamps(1) > vicon_timestamps(start_vicon))
    start_vicon = start_vicon + 1;
end

% normalize vicon trajectory to the start of the VO trajectory
current_vicon = start_vicon;
vicon_data_norm = cell(length(vicon_trajectories),1);
for i = 1:length(vo_timestamps)
    while(vo_timestamps(i) > vicon_timestamps(current_vicon))
        current_vicon = current_vicon + 1;
    end
    for j = 1:length(vicon_data_norm)
        vicon_data_norm{j}{end+1,1} = orthonormalize_svd((vicon_trajectories{j}{current_vicon}) * invT(vicon_trajectories{j}{start_vicon}));
    end
end

% plot vicon segment matching VO timestamps
plot_multiple_trajectories(vicon_data_norm)


% find the calibration for the VO, which should be similar to the provided
% T_to_apparatus_from_cam (T_AL)
T_vicon_from_vo = find_alignment_T_A_from_B(vicon_data_norm{end}, vo_data, true); 

% plot transformed VO segment with vicon segment
calibrated_vo = vo_data;
for i=1:length(vo_data)
    calibrated_vo{i} = (T_vicon_from_vo * vo_data{i} * invT(T_vicon_from_vo));
end
plot_multiple_trajectories([vicon_data_norm(end), {calibrated_vo}]);

% compare errors
[max_xyz_error, max_xyz_percent_error, max_rpy_error] = compare_estimate_to_ground_truth(vo_data, vicon_data_norm{end}, invT(T_vicon_from_vo))


