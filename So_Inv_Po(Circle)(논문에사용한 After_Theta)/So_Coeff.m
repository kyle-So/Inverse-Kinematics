function [ output_args ] = So_Coeff( input_args )
%SO_COEFF Summary of this function goes here
%   Detailed explanation goes here
global DOF L_end Coeff_att Coeff_rep
% DOF = 3
% L_end = 2

for i = 1 : DOF
    for j = 1: L_end
        if j == L_end
            Coeff_att(i,j) = i*j;
            Coeff_rep(i,j) = i*j;
        else 
            Coeff_att(i,j) = 1;
            Coeff_rep(i,j) = 1;       
        end
    end
end

% Coeff_att
% Coeff_rep
end

