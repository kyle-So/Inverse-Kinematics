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
                                   % ��ֹ��� �簢������ �����ؼ� ���� 2D ��ֹ� �ڵ�
                                   %                        Current_Dist_obs = [];
%                                                           Min_Dist_obs = [];
                            % % % % % % % % %         for ob_i = 1 : length( Transe_Obs-1)
                            % % % % % % % % %             Current_Dist_obs =   P_All{i,j}-Transe_Obs(:,ob_i); 
                            % % % % % % % % %             
                            % % % % % % % % %             if ob_i == 1 || (norm(Current_Dist_obs) < norm(Min_Dist_obs))
                            % % % % % % % % %                 D_Vec{i,j}= Current_Dist_obs;  % ��ֹ��� ������ �Ÿ��� ���Ͱ� xyz�� ��Ÿ��
                            % % % % % % % % %                 Min_Dist_obs = Current_Dist_obs;
                            % % % % % % % % %                 Min_obs_position{i,j} = Transe_Obs(:,ob_i);
                            % % % % % % % % %             end
                            % % % % % % % % %             
                            % % % % % % % % %         end
                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       

                             % ���� �� �Ÿ� ����.
                            D_Vec{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;


                        D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % �� �״�� �Ÿ� (������ġ-�߽�) - ������
%                           D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs});
                            if D_Norm{i,j,Num_Obs} <= Bound_con(Num_Obs)
                                Non_Demen_D_Norm=D_Norm{i,j,Num_Obs}/Bound_con(Num_Obs)
                                F_rep{i,j,Num_Obs} = 100*Coeff_rep(i,j)*( (1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) ) ).*(1/Non_Demen_D_Norm).^3 .* (D_Vec{i,j,Num_Obs});
%                             [i j Num_Obs]
%                             PM=(1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) )  % �ٿ�������� ������ ���, Ȥ�� ��ֹ����� ��� ���� ������ �׻� �о������
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

