function [T_to_apparatus_from_camera] = load_vicon_calibration(yaml_file)
%   
% Loads the calibrations transform from the camera to the vicon frame for
% the sensor apparatus.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   yaml_file: calibration file path  
%  
% output:
%   T_A_from_B: transform from the camera to the vicon frame for the sensor apparatus
%
    if(~contains(yaml_file, 'vicon') || ~endsWith(yaml_file, 'yaml'))
        warning('Are you sure this is a valid vicon.yaml file from the Oxford Multimotion Dataset?')
    end
    yaml_struct = ReadYaml(yaml_file);
    T_to_apparatus_from_camera = reshape(vertcat(yaml_struct.T_apparatus_left{:}),4,4);
end