function [ output_args ] = So_Coeff_0( input_args )
%SO_COEFF_0 Summary of this function goes here
%   Detailed explanation goes here
global DOF L_end Coeff_att Coeff_rep

for i = 1 : DOF
    for j = 1: L_end

            Coeff_att(i,j) = 1;
            Coeff_rep(i,j) = 1;       

    end
end

end

