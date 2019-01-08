function [ F_att,Unit_Vec_att  ] = So_F_att( Goal_All,P_All )
%SO_F_ATT Summary of this function goes here
%   Detailed explanation goes here

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att

    for i = 1 : DOF
       for j = 1 : length(mid_point)
            F_att{i,j}= Coeff_att(i,j).*(Goal_All{i,j}-P_All{i,j}) ;       % 2축이라 Z는 제거함
            Unit_Vec_att{i,j} = (Goal_All{i,j}-P_All{i,j})/norm((Goal_All{i,j}-P_All{i,j}));
            
% %             if ATT_POWER < norm(F_att{i,j})
% %                 ATT_POWER = norm(F_att{i,j});
% %             end
       end
    end

end

