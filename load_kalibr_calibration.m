function [camera_structs] = load_kalibr_calibration(yaml_file)
%   
% Loads the camera calibrations for the sensor % apparatus as estimated by
% kalibr.
%
% NOTE: Depends on yamlmatlab which is freely available at
%       https://code.google.com/archive/p/yamlmatlab/
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   yaml_file: calibration file path  
%  
% output:
%   camera_structs: camera parameters
%
    if(~contains(yaml_file, 'kalibr') || ~endsWith(yaml_file, 'yaml'))
        warning('Are you sure this is a valid kalibr.yaml file from the Oxford Multimotion Dataset?')
    end
    yaml_struct = ReadYaml(yaml_file);
    
    fields = fieldnames(yaml_struct);

   
    camera_structs(numel(fields),1).T_to_c0_from_c = eye(4);
    for i = 1:numel(fields)
        tmp = yaml_struct.(fields{i});
        camera_structs(i).rostopic = tmp.rostopic;
        camera_structs(i).resolution = tmp.resolution;
        camera_structs(i).K = [tmp.intrinsics{1} 0 tmp.intrinsics{3};
                               0 tmp.intrinsics{2} tmp.intrinsics{4};
                               0 0 1];
        if(isfield(tmp,'T_cn_cnm1'))
            camera_structs(i).T_to_c0_from_c = invT(reshape(vertcat(tmp.T_cn_cnm1{:}),4,4) * invT(camera_structs(i-1).T_to_c0_from_c));
        else
            camera_structs(i).T_to_c0_from_c = eye(4);
        end
        camera_structs(i).P = [camera_structs(i).K * camera_structs(i).T_to_c0_from_c(1:3,:); 0 0 0 1];
        camera_structs(i).T_to_c_from_i = reshape(vertcat(tmp.T_cam_imu{:}),4,4);
    end
end