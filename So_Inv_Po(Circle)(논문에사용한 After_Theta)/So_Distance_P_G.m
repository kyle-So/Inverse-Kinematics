function [ Divide_Number, Judge_End ] = So_Distance_P_G( Ini_All,P_All,Goal_All )
%SO_DISTANCE_P_G Summary of this function goes here
%   Detailed explanation goes here

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF L_end Coeff_att
Sum_Norm_P_G=0;
Sum_Norm_Ini_G=0;
    for i = 1 : DOF
       for j = 1 : length(mid_point)
           
           % 현재 위치 -> 목표
            Vec_P_G{i,j}= (Goal_All{i,j}-P_All{i,j}) ;
            Norm_P_G{i,j} = norm(Vec_P_G{i,j});
            Sum_Norm_P_G = Sum_Norm_P_G + Norm_P_G{i,j};
            
            % 초기 위치 -> 목표
            Vec_Ini_G{i,j}= (Goal_All{i,j}-Ini_All{i,j}) ;
            Norm_Ini_G{i,j} = norm(Vec_Ini_G{i,j});
            Sum_Norm_Ini_G = Sum_Norm_Ini_G + Norm_Ini_G{i,j};
            
       end
    end

    Average_Norm_P_G = Sum_Norm_P_G/(DOF*L_end);
    Average_Norm_Ini_G = Sum_Norm_Ini_G/(DOF*L_end);
    
    Ratio = (Average_Norm_P_G/Average_Norm_Ini_G)*100;
    Divide_Number = round(Ratio);
    
    Judge_End = isnan(Average_Norm_P_G);  % NaN이 떴을때 결과값 1을 반환해줌
end

