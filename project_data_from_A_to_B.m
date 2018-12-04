function [data_in_B] = project_data_from_A_to_B(T_B_from_A, data_in_A)
%
% Plots the linear acceleration vector, and rotation quaternion as recorded
% by the imu.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   T_B_from_A: a 4x4 transformation matrix from Frame_A to Frame_B
%   data_in_A: an Mx4 matrix of homogeneous data points in Frame_A
%
% output:
%   data_in_A: an Mx4 matrix of homogeneous data points in Frame_B
%

data_in_B = (T_B_from_A * data_in_A')';


end