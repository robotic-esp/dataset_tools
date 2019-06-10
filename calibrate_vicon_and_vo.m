% load vo data
vo_data = {eye(4)}; % load in VO data
vo_timestamps_data = readtable('stereo.csv');
start_index = 100;
end_index = start_index + 923;
vo_timestamps = vo_timestamps_data{start_index:end_index,2} + vo_timestamps_data{start_index:end_index,3}/1e9; 
vo_data_norm = cell(size(vo_data));
for i = 2:length(vo_timestamps)
    vo_data_norm{i-1,1} = orthonormalize_svd(invT(vo_data{i}) * vo_data{start_index});
end

% load vicon data
vicon_data = readtable('vicon.csv');
vicon_objs = unique(vicon_data.object);
vicon_trajectory = cell(length(vicon_objs),1);

w = waitbar(0, '');
for i=1:size(vicon_data,1)
    ind = find(contains(vicon_objs, vicon_data{i,3}));
    vicon_trajectory{ind}{end+1,1} = reshape(vicon_data{i,11:end}',4,4)';
    waitbar(i/size(vicon_data,1),w, '');
end
close(w);
vicon_timestamps = vicon_data{1:length(vicon_trajectory):end,1} + vicon_data{1:length(vicon_trajectory):end,2}/1e9;

% plot vicon data
to_plot = cell(size(vicon_trajectory));
for j = 1:length(vicon_trajectory)
    to_plot{j} = zeros(4,length(vicon_trajectory{j}));
    for i=1:length(vicon_trajectory{j})
        T_W_from_S_j = vicon_trajectory{j}{i};
        T_S_j_from_W = invT(T_W_from_S_j);
        to_plot{j}(:,i) = get_r_j_from_i_in_i_FROM_T_ji(T_S_j_from_W);
    end
end
figure(); hold on;
for j = 1:length(vicon_trajectory)
    plot3(to_plot{j}(1,:),to_plot{j}(2,:),to_plot{j}(3,:))
end
axis equal

% find vicon transforms corresponding to VO timestamps
start_vicon = 1;
while(vo_timestamps(1) > vicon_timestamps(start_vicon))
    start_vicon = start_vicon + 1;
end

current_vicon = start_vicon;
vicon_data_norm = cell(length(vicon_trajectory),1);
for i = 1:length(vo_timestamps)-1
    while(vo_timestamps(i) > vicon_timestamps(current_vicon))
        current_vicon = current_vicon + 1;
    end
    for j = 1:length(vicon_data_norm)
        vicon_data_norm{j}{end+1,1} = orthonormalize_svd(invT(vicon_trajectory{j}{current_vicon}) * vicon_trajectory{j}{start_vicon});
    end
end

% plot vicon segment matching VO timestamps
to_plot = cell(size(vicon_data_norm));
for j = 1:length(vicon_data_norm)
    to_plot{j} = zeros(4,length(vicon_data_norm{j}));
    for i=1:length(vicon_data_norm{j})
        to_plot{j}(:,i) = get_r_j_from_i_in_i_FROM_T_ji(vicon_data_norm{j}{i});
    end
end
figure(); hold on;
for j = 1:length(vicon_data_norm)
    plot3(to_plot{j}(1,:),to_plot{j}(2,:),to_plot{j}(3,:))
end
axis equal
to_plot = cell(size(vo_data_norm));
for j = 1:length(vo_data_norm)
    to_plot{j} = zeros(4,length(vo_data_norm{j}));
    for i=1:length(vo_data_norm{j})
        to_plot{j}(:,i) = get_r_j_from_i_in_i_FROM_T_ji(vo_data_norm{j}{i});
    end
end
for j = 1:length(vo_data_norm)
    plot3(to_plot{j}(1,:),to_plot{j}(2,:),to_plot{j}(3,:),'r')
end
axis equal
T_vicon_from_vo = find_alignment_T_A_from_B(vicon_data_norm{1}, vo_data_norm{1}, true);

% plot transformed vicon segment with VO segment
to_plot = cell(size(vo_data_norm));
for j = 1:length(vo_data_norm)
    to_plot{j} = zeros(4,length(vo_data_norm{j}));
    for i=1:length(vo_data_norm{j})
        to_plot{j}(:,i) = get_r_j_from_i_in_i_FROM_T_ji(T_vicon_from_vo * vo_data_norm{j}{i} * invT(T_vicon_from_vo));
    end
end
hold on;
for j = 1:length(vo_data_norm)
    plot3(to_plot{j}(1,:),to_plot{j}(2,:),to_plot{j}(3,:),'g')
end
axis equal

