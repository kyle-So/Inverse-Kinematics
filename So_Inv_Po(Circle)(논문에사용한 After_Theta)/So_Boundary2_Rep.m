function [ F_rep_2 ] = So_Boundary2_Rep( Goal_All, P_All, F_att, Unit_Vec_att  )
%SO_BOUNDARY2_REP Summary of this function goes here
%   Detailed explanation goes here
global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Bound_con_2 Coeff_rep


%%%%%%%%%%%%%%%%%% �� �̻��� ���� �ʿ�

       % 2�� �� Rep ��  ATT �̿��ظ��簪
    for i = 1 : DOF
           for j = 1 : length(mid_point)
%                F_rep{i,j}=[0,0,0];
               D_Vec_2{i,j}=[0;0;0];
               
               for Num_Obs = 1 : length(Obs_Circle)
                   Temp_F_rep_2 =[0;0;0];
                            % ���� �� �Ÿ� ����.
                            D_Vec_2{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;

                         Line_rep_2{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs}/norm(D_Vec_2{i,j,Num_Obs});       % P Obs ���͸� �������ͷ� �������
                        
                        D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % �� �״�� �Ÿ� (������ġ-�߽�) - ������
                        Bound_Circle(Num_Obs) = Bound_con(Num_Obs) + Obs_Cir_Radius(Num_Obs) ;             % ��輱1
                        Bound_Circle_2(Num_Obs) = Bound_con_2(Num_Obs) + Obs_Cir_Radius(Num_Obs);          % ��輱2


                        
                            if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
                             
                                if D_Norm_2{i,j,Num_Obs} < Bound_con(Num_Obs)
                                    Current_Coff_2{i,j,Num_Obs} = 0.5;
                                else
                                    Current_Coff_2{i,j,Num_Obs} = (( Bound_con_2(Num_Obs)-D_Norm_2{i,j,Num_Obs} )/( Bound_con_2(Num_Obs)-Bound_con(Num_Obs) ))/2;  %��輱 2���� �۾����� ���������� ���� 0~1 ����. 
                                end                                                                                                                  % 1���� ũ�ٴ¼Ҹ��� ���1���� �� �������ǹ�
                                Part_F_rep_2{i,j,Num_Obs} = Current_Coff_2{i,j,Num_Obs}.* norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
%                                 Temp_F_rep_2 = Temp_F_rep_2 + Current_Coff_2{i,j,Num_Obs}.*norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});    % att���� �̿��Ͽ� ���� �ݷ�
                                Temp_F_rep_2 = 200*Current_Coff_2{i,j,Num_Obs}*norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});    % att���� �̿��Ͽ� ���� �ݷ�

                            else
                                
                                Current_Coff_2{i,j,Num_Obs} = [0];
                                Part_F_rep_2{i,j,Num_Obs} = [0;0;0];
                                Temp_F_rep_2 = Temp_F_rep_2 + [0;0;0];        % Temp_F�� �ǹ̴� ��ֹ��� 2�� �̻�������, �� ��ֹ� ��ΰ���� �ݷ��� ��
                            end
                            F_rep_2{i,j,Num_Obs}=  Temp_F_rep_2;


               end
           end
    end

%     CheckVEC=cell2mat(D_Vec_2)
%     CheckDist=cell2mat(D_Norm_2)
%     FREP2=cell2mat(F_rep_2)
%     Norm_AT_RF = [norm(F_att) norm(F_rep_2)]
    
% % %                                     % Att�� ���̱�
% % %             for i = 1 : DOF
% % %                  for j = 1 : length(mid_point)
% % %                      if min(min(cell2mat(D_Norm_2))) <= Bound_con_2(Num_Obs)
% % %                         if Line_Theta{i,j,Num_Obs} >= 90  %�� ���������� ������ 90���� ũ�� -> ���������� �ƴ϶��. 
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

