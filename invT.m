function [Tji] = invT(Tij)
%   
% Efficiently computes the blockwise inverse of the SE(3) transform Tij.
%
% From: Kevin M. Judd and Jonathan D. Gammell, 
%       The Oxford Multimotion Dataset: Multiple SE(3) Motions with Ground Truth
%       kjudd@robots.ox.ac.uk, gammell@robots.ox.ac.uk
%
% input:
%   Tij: SE(3) transform to be inverted
%
% output:
%   Tji: inverse transform of Tji
%

    Cij = Tij(1:end-1,1:end-1);
    Cji = Cij';
    r_j_to_i_in_i = Tij(1:end-1,end);
    
    r_i_to_j_in_i = -Cji*r_j_to_i_in_i;
    
    Tji = [Cji r_i_to_j_in_i; 0 0 0 1];
end