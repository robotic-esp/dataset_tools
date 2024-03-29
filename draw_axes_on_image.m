%
% Plots the 3-D axes for each trajectory on the associated image.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk

clear;

% User must set these paths
path_to_vicon_data = ''; % path to vicon.csv file
path_to_vicon_calibration = ''; % path to vicon.yaml calibration file
% path_to_kalibr_calibration = '';% path to kalibr.yaml calibration file
path_to_manufacturer_calibration = ''; % path to manufacturer.yaml calibration file
path_to_images = ''; % path to the directory containing the stereo RGB images


[vicon_trajectories, vicon_objects, vicon_timestamps] = ingest_vicon_data(path_to_vicon_data);
T_to_apparatus_from_left_camera = load_vicon_calibration(path_to_vicon_calibration);
timestamps = readtable([path_to_images '/stereo.csv']);
timestamps = timestamps{:,2} + timestamps{:,3}/1e9; 


% Projects axes into left stereo image frame. Other image frames can be
% selected from the calibration.

% camera_structs_kalibr = load_kalibr_calibration(path_to_kalibr_calibration);
% T_to_left_camera_from_current_camera = camera_structs_kalibr(1).T_to_c0_from_c;
% fu = camera_structs_kalibr(1).K(1,1);
% fv = camera_structs_kalibr(1).K(2,2);
% Cu = camera_structs_kalibr(1).K(1,3);
% Cv = camera_structs_kalibr(1).K(2,3);
% baseline = camera_structs_kalibr(1).K(2,2);

camera_structs_manufacturer = load_manufacturer_calibration(path_to_manufacturer_calibration);
T_to_left_camera_from_current_camera = camera_structs_manufacturer(2).T_to_c0_from_c;
fu = camera_structs_manufacturer(2).K(1,1);
fv = camera_structs_manufacturer(2).K(2,2);
Cu = camera_structs_manufacturer(2).K(1,3);
Cv = camera_structs_manufacturer(2).K(2,3);
baseline = camera_structs_manufacturer(2).K(2,2);

images_format = [path_to_images '/%06d_left.png']; 
T_to_apparatus_from_current_camera = T_to_apparatus_from_left_camera * T_to_left_camera_from_current_camera;
P_uvd_to_xyz = [1/fu 0 0 -Cu/fu; ...
                0 1/fv 0 -Cv/fv; ...
                0 0 0 1; ...
                0 0 1/baseline 0];
P_xyz_to_uvd = inv(P_uvd_to_xyz);

figure(1);
for index = 1:1000
    vicon_index = 1;
    while(vicon_timestamps(vicon_index) < timestamps(index+1))
        vicon_index = vicon_index + 1;
    end

    clf;
    imshow(imread(sprintf(images_format, index)));
    for j = 1:length(vicon_objects)-1
        T_to_object_from_apparatus = vicon_trajectories{j}{vicon_index} * invT(vicon_trajectories{end}{vicon_index});
        T_to_object_from_camera = T_to_object_from_apparatus * T_to_apparatus_from_current_camera;
        plot_uvd_axes(T_to_object_from_camera, P_xyz_to_uvd)
    end
end

function [] = plot_uvd_axes(Tji, P_xyz_to_uvd)
scale = 0.5;
thickness = 5;

orig_axes = scale * [1 0 0;
             0 -1 0;
             0 0 1];

new_axes = [Tji(1:3,1:3)'*orig_axes; ones(1,3)];
new_axes(3,:) = new_axes(3,:) + 4;
new_axes_uvd = P_xyz_to_uvd * new_axes; 
new_axes_uvd = new_axes_uvd./new_axes_uvd(end,:);

centroid = get_r_j_from_i_in_i_FROM_T_ji(Tji);
centroid_uvd = P_xyz_to_uvd * centroid; 
centroid_uvd = centroid_uvd./centroid_uvd(end,:);
new_axes_uvd = new_axes_uvd + repmat(centroid_uvd,1,3) - repmat(P_xyz_to_uvd(:,3),1,3);

hold 'on'
plot([centroid_uvd(1) new_axes_uvd(1,1)], [centroid_uvd(2) new_axes_uvd(2,1)], 'Color', 'r', ...
         'LineWidth', thickness);
plot([centroid_uvd(1) new_axes_uvd(1,2)], [centroid_uvd(2) new_axes_uvd(2,2)], 'Color', 'g', ...
         'LineWidth', thickness);
plot([centroid_uvd(1) new_axes_uvd(1,3)], [centroid_uvd(2) new_axes_uvd(2,3)], 'Color', 'b', ...
         'LineWidth', thickness);
drawnow;
end


function [r_j_from_i_in_i] = get_r_j_from_i_in_i_FROM_T_ji(T_ji)

    r_i_from_j_in_j = T_ji(1:3,4);
    
    C_ji = T_ji(1:3,1:3);
    r_j_from_i_in_i = -C_ji' * r_i_from_j_in_j;
    r_j_from_i_in_i = [r_j_from_i_in_i;1];
end
