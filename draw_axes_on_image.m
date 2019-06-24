%
% Plots the 3-D axes for each trajectory on the associated image.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk


[vicon_trajectories, vicon_timestamps] = ingest_vicon_data(path_to_vicon_data);
T_to_apparatus_from_left_camera = load_vicon_calibration(path_to_vicon_calibration);
camera_structs_kalibr = load_kalibr_calibration(path_to_kalibr_calibration);
camera_structs_manufacturer = load_kalibr_calibration(path_to_manufacturer_calibration);
timestamps = readtable([path_to_images '/stereo.csv']);
timestamps = timestamps{:,2} + timestamps{:,3}/1e9; 


% Projects axes into left stereo image frame. Other image frames can be
% selected from the calibration.
T_to_left_camera_from_current_camera = camera_structs_kalibr(1).T_to_c0_from_c;
images_format = [path_to_images '/%06d_left.png']; 
T_to_apparatus_from_current_camera = T_to_apparatus_from_left_camera * T_to_left_camera_from_current_camera;
fu = camera_structs_kalibr(1).K(1,1);
fv = camera_structs_kalibr(1).K(2,2);
Cu = camera_structs_kalibr(1).K(1,3);
Cv = camera_structs_kalibr(1).K(2,3);
baseline = camera_structs_kalibr(1).K(2,2);
P_uvd_to_xyz = [1/fu 0 0 -Cu/fu; ...
                0 1/fv 0 -Cv/fv; ...
                0 0 0 1; ...
                0 0 1/baseline 0];
P_xyz_to_uvd = inv(P_uvd_to_xyz);

figure(1);
for index = 1:1000
    im = imread(sprintf(images_format, index));
    vicon_index = 1;
    while(vicon_timestamps(vicon_index) < timestamps(index+1))
        vicon_index = vicon_index + 1;
    end

    clf;imshow(imread(sprintf(images_format, index)));
    for j = 1:length(object_names)-1
        T_to_object_from_apparatus = invT(vicon_trajectories{j}{vicon_index}) * (vicon_trajectories{end}{vicon_index});
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
