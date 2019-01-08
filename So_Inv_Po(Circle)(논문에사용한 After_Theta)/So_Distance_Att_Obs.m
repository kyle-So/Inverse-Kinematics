function [ Norm_Q_to_Obs, Switch_F_Rep ] = So_Distance_Att_Obs( Goal_All, P_All )
%SO_DISTANCE_ATT_OBS Summary of this function goes here
%   Detailed explanation goes here
%   �κ����� ���� ��ġ -> ��ǥ��ġ�� ���Ϳ� 
%   ��ֹ� Obs������ �ִ� �Ÿ�
%   �̸� ���ϴ� ������ �κ����� �����̴� ��ο� ��ֹ��� �����ϴ°� �Ǻ�
%   �ִܰŸ� d�� ��ֹ��� ������ r���� �۴ٸ� �κ����� �̵���ο� ��ֹ��� �浹
%   Norm_Q_to_Obs : ������ġ�� ��ǥ�� ���Ϳ� ��ֹ� ������ �� ������ ���� ���� �ִܰŸ� d
%   Switch_F_Rep  : ô�½���ġ 1�̸� ô�� On, 0�̸� off. ������� ��ֹ��� 2���� [1 0] �� ǥ�� �̰��
%   ù��° ��ֹ��� On, �ι�°�� Off

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF

            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)
% i= DOF
% j=length(mid_point)
% Num_Obs = 1
                            Vec_P_to_Goal{i,j} =  Goal_All{i,j}-P_All{i,j};        %���� �κ��� ��ġ P ���� ��ǥ�� G����
                            Vec_P_to_Obs{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-P_All{i,j}; %���� �κ��� ��ġ P ���� ��ֹ� Obs����
                           
                            %���翵 ����ü
                            Proj_d{i,j} =    ( ( dot(Vec_P_to_Goal{i,j},Vec_P_to_Obs{i,j,Num_Obs}) )  / norm(Vec_P_to_Goal{i,j}) ) .* ( Vec_P_to_Goal{i,j}./norm(Vec_P_to_Goal{i,j}) );
                            Vec_Q_to_Obs{i,j,Num_Obs} =  Vec_P_to_Obs{i,j,Num_Obs} - Proj_d{i,j}; % ���翵���� ���� �����ǹ� Q�� ��ֹ��� ����
                            Norm_Q_to_Obs{i,j,Num_Obs} = norm(Vec_Q_to_Obs{i,j,Num_Obs});         % �����ǹ� QO�� �Ÿ�
                            
                        end
                    end
            end
            

                    %% �Ű����� t ã�� (t�� ������ �κ��Ϻ��� �ڿ��ִٴ� ��)
            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)

                            T_Vec_P_to_Goal{i,j} =  Goal_All{i,j}-P_All{i,j};        %���� �κ��� ��ġ P ���� ��ǥ�� G����
                            T_Vec_P_to_Obs{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-P_All{i,j}; %���� �κ��� ��ġ P ���� ��ֹ� Obs����
                           
                            parameter_t{i,j,Num_Obs} = (  ( P_All{i,j}(1)-Obs_Circle{Num_Obs}(1) )*(P_All{i,j}(1)-Goal_All{i,j}(1)) + ( P_All{i,j}(2)-Obs_Circle{Num_Obs}(2) )*(P_All{i,j}(2)-Goal_All{i,j}(2)) + ( P_All{i,j}(3)-Obs_Circle{Num_Obs}(3) )*(P_All{i,j}(3)-Goal_All{i,j}(3))  ) ./ norm(T_Vec_P_to_Goal{i,j}).^2  ;
                        end
                    end
            end
            
            
            
%% ��ֹ��� ��θ� �����ϳ� ���ϳ� ����

                        %�Ǻ���� �����
                        for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                     if Norm_Q_to_Obs{i,j,Num_Obs} < 1.5*Obs_Cir_Radius(Num_Obs)      % ���� Att�� ��ֹ��� �ִܰŸ��� ��ֹ� ���������� �۴ٸ�
                                        Judge_Norm_Q_to_Obs{i,j,Num_Obs} = 1;                     % �浹 ���ɼ��� �����Ƿ� �浹 on = 1
                                     else
                                        Judge_Norm_Q_to_Obs{i,j,Num_Obs} = 0;    
                                     end
                                 
                                     if and(parameter_t{i,j,Num_Obs} >= 0 , parameter_t{i,j,Num_Obs} <= 1)  % �Ķ���Ͱ� t�� 0~1���̿� �����ؾ� ��ֹ��� �̵���λ� ����
                                         Judge_parameter_t{i,j,Num_Obs} = 1;                                % ���� 0~1���̿� ������ �浹 on = 1
                                     else
                                        Judge_parameter_t{i,j,Num_Obs} = 0;
                                     end
                                 end
                             end
                        end
                        

                        % �Ǻ���� �� ���� ������ ��� 1�� ������ ��ֹ��� �̵���ο� �����Ƿ� �Ǻ������ ����
                        % �̿��� �Ѵ� �����ϴ��� Ȯ��.
                      
                                 for Num_Obs = 1 : length(Obs_Circle)
                                    Judge_Matrix{:,:,Num_Obs} = cell2mat(Judge_Norm_Q_to_Obs(:,:,Num_Obs)) .* cell2mat(Judge_parameter_t(:,:,Num_Obs));
% % % % %                                     if and(min(min(cell2mat(parameter_t(:,:,Num_Obs)))) < 0, max(max(cell2mat(parameter_t(:,:,Num_Obs)))) > 1)     % ���� ���� ���� ������ �� ��ֹ��� �κ��Ⱥ��� �ڿ� ����
% % % % %                                         Switch_F_Rep(Num_Obs) = 0;                                  % ��ֹ��� �κ��Ⱥ��� �ڿ� �����Ƿ� ��ǥ������ ���µ� ���ذ��ȵ�
% % % % %                                     else
% % % % %                                         if min(min(cell2mat(Norm_Q_to_Obs(:,:,Num_Obs)))) < Obs_Cir_Radius(Num_Obs)       % �ִܰŸ��� ���� ���ǵ� ������ r ���� �۴ٸ�,
% % % % %                                             Switch_F_Rep(Num_Obs) = 1;                     % ��ֹ��� �浹�ϹǷ� ô�� �ʿ�
% % % % %                                         else
% % % % %                                             Switch_F_Rep(Num_Obs) = 0;                     % ��ֹ��� �κ��Ⱥ��ٴ� ������ ������ �浹������ ����
% % % % %                                         end
% % % % %                                     end
                                 end
                                 
                                 for Num_Obs = 1 : length(Obs_Circle)                                
                                    if max(max(cell2mat(Judge_Matrix(Num_Obs)))) == 1
                                        Switch_F_Rep(Num_Obs) = 1;
                                    else
                                        Switch_F_Rep(Num_Obs) = 0;
                                    end
                                 end
% MAXCHECK= max(max(cell2mat(Judge_Matrix(1))))

% % Switch_F_Rep
% % % % % 
% % % % %                         
% %                       parameter_t   % �Ű�����
% % Norm_Q_to_Obs
% % cell2mat(Judge_Matrix)
% % 
% % Judge_Norm_Q_to_Obs
% % 
% % Judge_parameter_t

end

