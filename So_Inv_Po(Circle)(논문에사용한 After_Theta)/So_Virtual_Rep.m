function [ Ex_Sum_Torque_rep, Ex_F_rep ] = So_Virtual_Rep( P_All,J,Ex_Obs_Circle )
%SO_VIRTUAL_REP Summary of this function goes here
%   Detailed explanation goes here

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Bound_con_2 Coeff_rep Try_Num L_end
global Ex_Bound_con
Ex_Coef_F_rep = 10;
Ex_Sum_Torque_rep = zeros(DOF,1);

for i = 1 : DOF
     for j = 1 : length(mid_point)
           Ex_F_att{i,j}= [0;0;0] ;       %에러방지 초기값 설정
           Ex_F_rep{i,j}= [0;0;0] ;
     end
end 

        for i = 1 : DOF
           for j = 1 : length(mid_point)
%                Ex_F_rep{i,j}=[0,0,0];
               Ex_D_Vec{i,j}=[0;0;0];     %사용하기전 변수 초기화, 가상 장애물과 End-effect의 거래
               Ex_Temp_F_rep =[0;0;0];    % 가상 반력
           end
        end



         for i = DOF
           for j = 1 : length(mid_point)       
               for Ex_Obs_Order = 1 : length(Ex_Obs_Circle)

                             % 원일 때 거리 측정.
                         for Cir_i = 1 : DOF
                            for Cir_j =1 : length(mid_point)
                                Ex_D_Vec{Cir_i,Cir_j,Ex_Obs_Order} = P_All{Cir_i,Cir_j}-Ex_Obs_Circle{Ex_Obs_Order}' ;
                            end
                         end

                            Ex_D_Norm{i,j,Ex_Obs_Order} = norm(Ex_D_Vec{i,j,Ex_Obs_Order});       % 말 그대로 거리 (현재위치-중심)

                        if Ex_D_Norm{i,j,Ex_Obs_Order} <= Ex_Bound_con    %Ex_Bound_con은 임시... 장애물에 따른 설정필요할듯
                            Ex_Part_F_rep{i,j,Ex_Obs_Order} = Ex_Coef_F_rep*( (1/Ex_D_Norm{i,j,Ex_Obs_Order}) - ( 1/Ex_Bound_con ) ).*(1/Ex_D_Norm{i,j,Ex_Obs_Order}).^3 .* (Ex_D_Vec{i,j,Ex_Obs_Order});
                            Ex_Temp_F_rep = Ex_Temp_F_rep + Ex_Coef_F_rep*( (1/Ex_D_Norm{i,j,Ex_Obs_Order}) - ( 1/Ex_Bound_con ) ).*(1/Ex_D_Norm{i,j,Ex_Obs_Order}).^3 .* (Ex_D_Vec{i,j,Ex_Obs_Order});
                        else
                            Ex_Part_F_rep{i,j,Ex_Obs_Order} = [0;0;0];
                            Ex_Temp_F_rep = Ex_Temp_F_rep + [0;0;0];
                        end
                        Ex_F_rep{i,j}= Ex_Temp_F_rep;


               end

                                             % Ex_Rep 힘-> 토크
                                           Ex_Torque_rep{i,j} = J{i,j}'*Ex_F_rep{i,j};
                                           Ex_Sum_Torque_rep = Ex_Sum_Torque_rep+Ex_Torque_rep{i,j};

           end
        end
end

