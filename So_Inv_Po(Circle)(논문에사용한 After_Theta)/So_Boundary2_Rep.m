function [ F_rep_2 ] = So_Boundary2_Rep( Goal_All, P_All, F_att, Unit_Vec_att  )
%SO_BOUNDARY2_REP Summary of this function goes here
%   Detailed explanation goes here
global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Bound_con_2 Coeff_rep


%%%%%%%%%%%%%%%%%% 좀 이상함 수정 필요

       % 2차 방어선 Rep 힘  ATT 이용해만든값
    for i = 1 : DOF
           for j = 1 : length(mid_point)
%                F_rep{i,j}=[0,0,0];
               D_Vec_2{i,j}=[0;0;0];
               
               for Num_Obs = 1 : length(Obs_Circle)
                   Temp_F_rep_2 =[0;0;0];
                            % 원일 때 거리 측정.
                            D_Vec_2{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;

                         Line_rep_2{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs}/norm(D_Vec_2{i,j,Num_Obs});       % P Obs 벡터를 단위벡터로 만들어줌
                        
                        D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % 말 그대로 거리 (현재위치-중심) - 반지름
                        Bound_Circle(Num_Obs) = Bound_con(Num_Obs) + Obs_Cir_Radius(Num_Obs) ;             % 경계선1
                        Bound_Circle_2(Num_Obs) = Bound_con_2(Num_Obs) + Obs_Cir_Radius(Num_Obs);          % 경계선2


                        
                            if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
                             
                                if D_Norm_2{i,j,Num_Obs} < Bound_con(Num_Obs)
                                    Current_Coff_2{i,j,Num_Obs} = 0.5;
                                else
                                    Current_Coff_2{i,j,Num_Obs} = (( Bound_con_2(Num_Obs)-D_Norm_2{i,j,Num_Obs} )/( Bound_con_2(Num_Obs)-Bound_con(Num_Obs) ))/2;  %경계선 2보다 작아지면 선형적으로 증가 0~1 까지. 
                                end                                                                                                                  % 1보다 크다는소리는 경계1보다 더 작은값의미
                                Part_F_rep_2{i,j,Num_Obs} = Current_Coff_2{i,j,Num_Obs}.* norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
%                                 Temp_F_rep_2 = Temp_F_rep_2 + Current_Coff_2{i,j,Num_Obs}.*norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});    % att힘을 이용하여 만든 반력
                                Temp_F_rep_2 = 200*Current_Coff_2{i,j,Num_Obs}*norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});    % att힘을 이용하여 만든 반력

                            else
                                
                                Current_Coff_2{i,j,Num_Obs} = [0];
                                Part_F_rep_2{i,j,Num_Obs} = [0;0;0];
                                Temp_F_rep_2 = Temp_F_rep_2 + [0;0;0];        % Temp_F의 의미는 장애물이 2개 이상있을때, 두 장애물 모두고려한 반력의 합
                            end
                            F_rep_2{i,j,Num_Obs}=  Temp_F_rep_2;


               end
           end
    end

%     CheckVEC=cell2mat(D_Vec_2)
%     CheckDist=cell2mat(D_Norm_2)
%     FREP2=cell2mat(F_rep_2)
%     Norm_AT_RF = [norm(F_att) norm(F_rep_2)]
    
% % %                                     % Att힘 줄이기
% % %             for i = 1 : DOF
% % %                  for j = 1 : length(mid_point)
% % %                      if min(min(cell2mat(D_Norm_2))) <= Bound_con_2(Num_Obs)
% % %                         if Line_Theta{i,j,Num_Obs} >= 90  %두 직선사이의 각도가 90보다 크면 -> 같은방향이 아니라면. 
% % %                             F_att_Reverse{i,j}= 1*(0.2+Current_Coff_2{i,j,Num_Obs}).*F_att{i,j};
% % %                         end
% % %                      else
% % %                           F_att_Reverse{i,j}=  [0;0;0];
% % %                      end
% % % 
% % % % %                      if ATT_POWER_Re < norm(F_att_Reverse{i,j})
% % % % %                         ATT_POWER_Re = norm(F_att_Reverse{i,j});
% % % % %                      end
% % %                  end
% % %             end



end

