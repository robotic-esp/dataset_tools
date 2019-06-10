function T_A_from_B = find_alignment_T_A_from_B(transforms_A, transforms_B, allow_translation)
%   
% Determines the alignment transform that minimizes the sum of errors
% between each pair of transforms in A and B.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   transforms_A: a Kx1 cell array of 4x4 transformation matrices
%   representing a discrete motion trajectory
%   transforms_B: a Kx1 cell array of 4x4 transformation matrices
%   representing a discrete motion trajectory
%   allow_translation: logical governing whether the alignment transform
%   may include translational components, rather than just rotational
%
% output:
%   T_A_from_B: alignment transform that minimizes the accumulated SE(3)
%   error between trajectories A and B
%

if length(transforms_A) ~= length(transforms_B)
    error('ERROR - mismatched lengths');
elseif (~isa(transforms_A, 'cell') || ~isa(transforms_B, 'cell'))
    error('ERROR - inputs 1 and 2 must be cell arrays');
elseif nargin == 3 && ~isa(allow_translation, 'logical')
    error('ERROR - input 3 must be a logical (true/false)');
elseif nargin == 2
    allow_translation = true;
end

error_se3 = @(T_A_from_B) 0.5*(([0 0 0 1 1 1]' .* SE3_to_se3_lie(T_A_from_B))' * ([0 0 0 1 1 1]' .* SE3_to_se3_lie(T_A_from_B)));
error_function = @(T_A_from_B, T_A, T_B) error_se3(T_A_from_B * T_B * invT(T_A_from_B) * invT(T_A));
accum_error_se3 = @(xi_A_from_B, T_A_cell, T_B_cell) sum(cellfun(@(T_A, T_B) error_function(se3_lie_to_SE3(xi_A_from_B), T_A, T_B), T_A_cell, T_B_cell));

xi = [0 0 0 0 0 0]'; % identity
options = optimoptions(@fminunc,'Display','off','Algorithm','quasi-newton');
to_minimize = @(x) accum_error_se3(x, transforms_A, transforms_B);
xi = fminunc(to_minimize, xi, options);

if(allow_translation)
    error_se3 = @(T_A_from_B) 0.5*(SE3_to_se3_lie(T_A_from_B)' * SE3_to_se3_lie(T_A_from_B));
    error_function = @(T_A_from_B, T_A, T_B) error_se3(T_A_from_B * T_B * invT(T_A_from_B) * invT(T_A));
    accum_error_se3 = @(xi_A_from_B, T_A_cell, T_B_cell) sum(cellfun(@(T_A, T_B) error_function(se3_lie_to_SE3(xi_A_from_B), T_A, T_B), T_A_cell, T_B_cell));
    to_minimize = @(x) accum_error_se3(x, transforms_A, transforms_B);
    xi = fminunc(to_minimize, xi, options);
end

T_A_from_B = se3_lie_to_SE3(xi);
end

function [ xi ] = SE3_to_se3_lie( T )
%SE3_TO_se3 Summary of this function goes here
%   Detailed explanation goes here

    [v,e] = eig(T(1:3,1:3));
    a = real(acos((trace(T(1:3,1:3))-1)/2)*v(:,find(abs(imag(diag(e))) < 1e-10)));
    a = a(:,1);
    phi = norm(a);
    if(phi < 1e-6)
        xi = zeros(6,1);
    else
        a = a/phi;
        J_inv = (phi/2)*cot(phi/2)*eye(3) + (1-(phi/2)*cot(phi/2))*(a*a') - (phi/2)*skew_symmetric(a);
        rho = J_inv*T(1:3,end);
        xi = [rho; phi*a];
    end
    
end

function [ T ] = se3_lie_to_SE3( xi )
%SE3_TO_SE3 Summary of this function goes here
%   Detailed explanation goes here

    phi = norm(xi(4:6));
    if(phi <= 1e-12)
        T = expm(skew_symmetric(xi));
    else
        a = xi(4:6)/phi; 
        C = cos(phi) * eye(3) + (1 - cos(phi)) * a * a' + sin(phi) * skew_symmetric(a);
        J = (sin(phi)/phi)*eye(3) + (1-sin(phi)/phi)*a*a' + ((1-cos(phi))/phi)*skew_symmetric(a);
        r = J*xi(1:3);
        T = [C r; 0 0 0 1];
    end
end

function [x_caret] = skew_symmetric(x)
    if size(x,1) == 1
        x = x';
    end
    if size(x,1) == 3
        x_caret = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    elseif size(x,1) == 6
        u = x(1:3);
        v = x(4:6);
        x_caret = [skew_symmetric(v) u; zeros(1,4)];
    else
        %throw error
    end
end
