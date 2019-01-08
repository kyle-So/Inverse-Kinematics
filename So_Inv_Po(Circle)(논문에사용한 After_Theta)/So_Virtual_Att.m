function [ Ex_Sum_Torque_att,Ex_F_att  ] = So_Virtual_Att( P_All,J )
%SO_VIRTUAL_ATT Summary of this function goes here
%   Detailed explanation goes here
global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Bound_con_2 Coeff_rep Try_Num L_end
global Ex_Bound_con


Ex_Sum_Torque_att = zeros(DOF,1);

        for i = 1 : DOF
           for j = 1 : length(mid_point)
                Ex_F_att{i,j}= 10*Coeff_att(2*(i-1)+j)*(-P_All{i,j}) ;       % 2축이라 Z는 제거함
                              % Ex_Att 힘-> 토크
                           Ex_Torque_att{i,j} = J{i,j}'*Ex_F_att{i,j};
                           Ex_Sum_Torque_att = Ex_Sum_Torque_att+Ex_Torque_att{i,j};
           end
        end

end

