function [ F_rep ] = So_F_rep( Goal_All,P_All )
%SO_F_REP Summary of this function goes here
%   Detailed explanation goes here
global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Coeff_rep

    for i = 1 : DOF
           for j = 1 : length(mid_point)
%                F_rep{i,j}=[0,0,0];
               D_Vec{i,j}=[0;0;0];
               
               for Num_Obs = 1 : length(Obs_Circle)
                    Temp_F_rep{i,j,Num_Obs} =[0;0;0];
                       
                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                   % 장애물을 사각형으로 생각해서 만든 2D 장애물 코드
                                   %                        Current_Dist_obs = [];
%                                                           Min_Dist_obs = [];
                            % % % % % % % % %         for ob_i = 1 : length( Transe_Obs-1)
                            % % % % % % % % %             Current_Dist_obs =   P_All{i,j}-Transe_Obs(:,ob_i); 
                            % % % % % % % % %             
                            % % % % % % % % %             if ob_i == 1 || (norm(Current_Dist_obs) < norm(Min_Dist_obs))
                            % % % % % % % % %                 D_Vec{i,j}= Current_Dist_obs;  % 장애물과 현재의 거리를 벡터값 xyz로 나타냄
                            % % % % % % % % %                 Min_Dist_obs = Current_Dist_obs;
                            % % % % % % % % %                 Min_obs_position{i,j} = Transe_Obs(:,ob_i);
                            % % % % % % % % %             end
                            % % % % % % % % %             
                            % % % % % % % % %         end
                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       

                             % 원일 때 거리 측정.
                            D_Vec{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;


                        D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % 말 그대로 거리 (현재위치-중심) - 반지름
%                           D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs});
                            if D_Norm{i,j,Num_Obs} <= Bound_con(Num_Obs)
                                Non_Demen_D_Norm=D_Norm{i,j,Num_Obs}/Bound_con(Num_Obs)
                                F_rep{i,j,Num_Obs} = 100*Coeff_rep(i,j)*( (1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) ) ).*(1/Non_Demen_D_Norm).^3 .* (D_Vec{i,j,Num_Obs});
%                             [i j Num_Obs]
%                             PM=(1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) )  % 바운더리보다 작으면 양수, 혹시 장애물보다 깊게 들어가면 음수로 항상 밀어내게해줌
%                             DN=D_Norm{i,j,Num_Obs}
%                             BC=Bound_con(Num_Obs)
%                             DN_BC=D_Norm{i,j,Num_Obs}/Bound_con(Num_Obs)
%                             POWER = 1/DN_BC^3
                            else
                                F_rep{i,j,Num_Obs} = [0;0;0];       
                            end

               end
           end
    end
%             FR=cell2mat(F_rep)
%             Chcek_Norm=cell2mat(D_Norm)
end

