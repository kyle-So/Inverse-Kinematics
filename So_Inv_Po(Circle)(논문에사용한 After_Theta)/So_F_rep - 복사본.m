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
                         for Cir_i = 1 : DOF
                            for Cir_j =1 : length(mid_point)
                                D_Vec{Cir_i,Cir_j,Num_Obs} = P_All{Cir_i,Cir_j}-Obs_Circle{Num_Obs}' ;
                            end
                         end


                        D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % 말 그대로 거리 (현재위치-중심) - 반지름
%                           D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs});
                            if D_Norm{i,j,Num_Obs} <= Bound_con(Num_Obs)
                                
                                Part_F_rep{i,j,Num_Obs} = Coeff_rep(i+j-1)*( (1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) ) ).*(1/D_Norm{i,j,Num_Obs}).^3 .* (D_Vec{i,j,Num_Obs});
                                Temp_F_rep{i,j,Num_Obs} = Temp_F_rep{i,j,Num_Obs} + Coeff_rep(i+j-1)*( (1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) ) ).*(1/D_Norm{i,j,Num_Obs}).^3 .* (D_Vec{i,j,Num_Obs});
                                
                            else
                                Part_F_rep{i,j,Num_Obs} = [0;0;0];
                                Temp_F_rep{i,j,Num_Obs} = Temp_F_rep{i,j,Num_Obs} + [0;0;0];        % Temp_F의 의미는 장애물이 2개 이상있을때, 두 장애물 모두고려한 반력의 합
                            end
                                    %지금보니까 Temp_로 더한다고 했는데 더하는 부분이 없는데?
                                    %i,j,Num_Obs의 값이 계속 달라지니까 중복되는값이 없어서 다
                                    %0+F_rep 값아닌가?
                            
                            F_rep{i,j,Num_Obs}=  Temp_F_rep{i,j,Num_Obs};
                            
% %                             if REP_POWER < norm(F_rep{i,j})
% %                                 REP_POWER = norm(F_rep{i,j});
% %                             end
                            
                            
               end
           end
    end
% Distance_Obs = cell2mat(D_Norm)

end

