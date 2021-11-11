function [rotation_quaternion, angular_velocity, linear_acceleration, imu_timestamps] = ingest_imu_data(filename)
%
% Parses IMU data from input csv into rotation, angular velocity, and
% linear acceleration matrices along with timestamp data.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   filename: input csv filename
%
% output:
%   rotation_quaternion: Kx4 matrix of rotation quaternion measurements
%   angular_velocity: Kx3 matrix of angular velocity measurements
%   linear_acceleration: Kx3 matrix of linear acceleration measurements
%   imu_timestamps: Kx1 vector of timestamps
%

% load imu data
imu_data = readtable(filename);
linear_acceleration = zeros(size(imu_data,1),3);
angular_velocity = zeros(size(imu_data,1),3);
rotation_quaternion = zeros(size(imu_data,1),4);

w = waitbar(0, '');
for i=1:size(imu_data,1)
    rotation_quaternion(i,1:4) = imu_data{i,3:6};
    angular_velocity(i,1:3) = imu_data{i,7:9};
    linear_acceleration(i,1:3) = imu_data{i,10:12};
    waitbar(i/size(imu_data,1),w, [num2str(i) '/' num2str(size(imu_data,1)) ' entries read']);
end
close(w);
imu_timestamps = imu_data{:,1} + imu_data{:,2}/1e9;

end