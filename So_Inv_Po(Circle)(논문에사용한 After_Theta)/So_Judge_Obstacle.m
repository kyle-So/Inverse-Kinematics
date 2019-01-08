function [ Switch_Obs,Norm_Q_to_Obs, Switch_F_Rep,Judge_Matrix_0,Judge_Matrix ] = So_Judge_Obstacle( Goal_All, P_All )
%SO_DISTANCE_ATT_OBS Summary of this function goes here
%   Detailed explanation goes here
%   �κ����� ���� ��ġ -> ��ǥ��ġ�� ���Ϳ� 
%   ��ֹ� Obs������ �ִ� �Ÿ�
%   �̸� ���ϴ� ������ �κ����� �����̴� ��ο� ��ֹ��� �����ϴ°� �Ǻ�
%   �ִܰŸ� d�� ��ֹ��� ������ r���� �۴ٸ� �κ����� �̵���ο� ��ֹ��� �浹

% Switch_Obs : �Ǻ���� Both_Judge�� ��� ���� 0�̸� Off, �ƴϸ� On
%		On(1) : ��λ� ��ֹ��� ����
%		Off(0) : ��λ� ��ֹ��� ����  
%   Norm_Q_to_Obs : ������ġ�� ��ǥ�� ���Ϳ� ��ֹ� ������ �� ������ ���� ���� �ִܰŸ� d
%   Switch_F_Rep  : ô�½���ġ 1�̸� ô�� On, 0�̸� off. ������� ��ֹ��� 2���� [1 0] �� ǥ�� �̰��
%   ù��° ��ֹ��� On, �ι�°�� Off
% Judge_Matrix_0 : �����ǹ��� �Ÿ� d�� ��ֹ� ���������� ���� ��, �Ű����� t�� 0~1�����϶� 
%  0<t<1, d < r �� ������ ��� ������ �� On
% Judge_Matrix_0 : �����ǹ��� �Ÿ� d�� ��ֹ� ������+Boundary ���� ���� ��, �Ű����� t�� 0~1�����϶� 
% 0<t<1, d < r+Boundary �� ������ ��� ������ �� On

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Bound_con_2

            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)
% i= DOF
% j=length(mid_point)
% Num_Obs = 1
                            Vec_P_to_Goal{i,j} =  Goal_All{i,j}-P_All{i,j};        %���� �κ��� ��ġ P ���� ��ǥ�� G����
                            Vec_P_to_Obs{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-P_All{i,j}; %���� �κ��� ��ġ P ���� ��ֹ� Obs����
                           
                            %���翵 ����ü
                            Proj_d{i,j,Num_Obs} =    ( ( dot(Vec_P_to_Goal{i,j},Vec_P_to_Obs{i,j,Num_Obs}) )  / norm(Vec_P_to_Goal{i,j}) ) .* ( Vec_P_to_Goal{i,j}./norm(Vec_P_to_Goal{i,j}) );
                            Vec_Q_to_Obs{i,j,Num_Obs} =  Vec_P_to_Obs{i,j,Num_Obs} - Proj_d{i,j,Num_Obs}; % ���翵���� ���� �����ǹ� Q�� ��ֹ��� ����
                            Norm_Q_to_Obs{i,j,Num_Obs} = norm(Vec_Q_to_Obs{i,j,Num_Obs});         % �����ǹ� QO�� �Ÿ�
                            
                        end
                    end
            end
            
            %% �ηϰ��� ���� �Ʒ� �Ű������� t norm(���翵)/norm(PG)���� �̿��ؼ� �����
              % t = norm(���翵)/norm(PG) �̿��� ����� �ְ� �� ���� 0~1���̰� �Ǿ���ϸ�
              % ���翵�� ���Ͱ� PG�� �ٸ��� �����̸� �̰��� ����� �ʿ���� �浹���Ͼ
              % �������� �ٽ� ������ �浹 ������ �Ű����� t�� 0~1���� ���̸鼭
              % ��(��ֹ�)�� ����(������ġ���� ��ǥ��ġ ����)������ �ִܰŸ� d�� ��ֹ��� ������r
              % ���� �������ǿ��� �浹�� ���ɼ��� �����Ƿ� �� ���� ô�� ������ On(1)�� �ȴ�.
              
              % ��¶�� �Ʒ� �Ű������� �������� ���翵�� �̿��ؼ� ���غ���
              
              
              for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)
                            Proj_parameter_t{i,j,Num_Obs} = norm(Proj_d{i,j,Num_Obs})./ norm(Vec_P_to_Goal{i,j});
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
            
            %�Ķ���� t�� ���� 0~1 ���̿� �����ؾ߸� ��ֹ��� ��λ� �ִٴ� �Ҹ�.
            
%% 0�� ��ֹ��� ��θ� �����ϳ� ���ϳ� ����

                        %�Ǻ���� �����
                        for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                     if Norm_Q_to_Obs{i,j,Num_Obs} < Obs_Cir_Radius(Num_Obs)      % ���� Att�� ��ֹ��� �ִܰŸ��� ��ֹ� ���������� �۴ٸ�
                                        Judge_Norm_Q_to_Obs_0(i,j,Num_Obs) = 1;                     % �浹 ���ɼ��� �����Ƿ� �浹 on = 1
                                     else
                                        Judge_Norm_Q_to_Obs_0(i,j,Num_Obs) = 0;    
                                     end
                                 
                                     if and(parameter_t{i,j,Num_Obs} >= 0 , parameter_t{i,j,Num_Obs} <= 1)  % �Ķ���Ͱ� t�� 0~1���̿� �����ؾ� ��ֹ��� �̵���λ� ����
                                         Judge_parameter_t_0(i,j,Num_Obs) = 1;                                % ���� 0~1���̿� ������ �浹 on = 1
                                     else
                                        Judge_parameter_t_0(i,j,Num_Obs) = 0;       %0���� 1���� ��� �̰� �����ϱ� ���� �ȹٲ㵵 �ɵ�.
                                     end
                                 end
                             end
                        end
                        

                        % �Ǻ���� �� ���� ������ ��� 1�� ������ ��ֹ��� �̵���ο� �����Ƿ� �Ǻ������ ����
                        % �̿��� �Ѵ� �����ϴ��� Ȯ��.
                          for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                    Judge_Matrix_0(i,j,Num_Obs) = (Judge_Norm_Q_to_Obs_0(i,j,Num_Obs)) .* (Judge_parameter_t_0(i,j,Num_Obs));

                                 end
                             end
                          end

                                 for Num_Obs = 1 : length(Obs_Circle)                                
                                    if max(max((Judge_Matrix_0(:,:,Num_Obs)))) == 1
                                        Switch_F_Rep_0(Num_Obs) = 1;
                                    else
                                        Switch_F_Rep_0(Num_Obs) = 0;
                                    end
                                 end

 %% ��ǥ��ġ�� ��ֹ� ������ ����,�׸��� ��谪

% �������� ���� ��ġ�� ��ֹ� ��θ� �����ٸ�,
% ���⼭�� ������ ��ġ�� ��ǥ��ġ�� ��ֹ��� �Ÿ��� �Ǵ��Ѵ�.
% ��ֹ��� ��ǥ��ġ�� ���� �����ϴٸ�, ��ֹ��� ��谪�ȿ� ��ǥ��ġ�� �����ϰ� �ǰ�
% �̴� ����ġ�� �׻� On�� �� �� �ۿ� �����Ƿ� ������ �η� ô�� ���� ������ ����ġ�� Off �ε�
% �̷� ��� ����ġ�� ������ �ʴ� ��찡 �����Ѵ�.
% �̰� ������ �ƴ����� �ϴ� ��ֹ� ��谪2�� 1.5r �̹Ƿ� �ּҰŸ��� 1.5r���ٴ� Ŀ���Ѵ�.



            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                    for Num_Obs = 1 : length(Obs_Circle)
                        Obs_Goal{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-Goal_All{i,j}; %���� �κ��� ��ġ P ���� ��ֹ� Obs����
                        Norm_Obs_Goal(i,j,Num_Obs) = norm(Obs_Goal{i,j,Num_Obs});
                    
% % %                             % �Ǵ���� �����
% % %                             % 2����輱���� ������ 0, �ƴϸ� 1? �ϴ���
% % %                             if Norm_Obs_Goal(i,j,Num_Obs) < 1.5*Obs_Cir_Radius(Num_Obs)
% % %                                 Judge_Norm_Obs_Goal(i,j,Num_Obs) = 0;
% % %                             else
% % %                                 Judge_Norm_Obs_Goal(i,j,Num_Obs) = 1;
% % %                             end       
                    
                    end
                 end
            end
%             Judge_Norm_Obs_Goal
            
            
            % �κ� ��ġ�� ��ֹ� ���� ����� �Ÿ�
            for Num_Obs = 1 : length(Obs_Circle)
                Minimum_Dis_G_Obs(Num_Obs) = min(min(Norm_Obs_Goal(:,:,Num_Obs)));
            end
%           Minimum_Dis_G_Obs
            % ��� �����ϰ� �ּҰŸ� ��
            for Num_Obs = 1 : length(Obs_Circle)
                if Switch_F_Rep_0(Num_Obs)  == 1        % ��ֹ��� ��λ� �����ϸ鼭 ��ǥ��ġ�� �����ϸ�
                                              % 1.5r ���� ��ǥ��ġ�� ���� �� �����Ƿ�
                                              % ��ǥ��ġ�� ���� ����� �Ÿ���ŭ���� ����
                    if Minimum_Dis_G_Obs(Num_Obs) < Bound_con_2(Num_Obs)    
                        disp('��谪����')
                        Bound_con_2(Num_Obs) = Minimum_Dis_G_Obs(Num_Obs)-0.2
                    end
                end
            end               
                            
                                 
                                 
%% 2�� Boundary�� ��ֹ��� ��θ� �����ϳ� ���ϳ� ����

                        %�Ǻ���� �����
                        for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                     if Norm_Q_to_Obs{i,j,Num_Obs} < (Bound_con_2(Num_Obs)+Obs_Cir_Radius(Num_Obs))      % ���� Att�� ��ֹ��� �ִܰŸ��� ��ֹ� ���������� �۴ٸ�
                                        Judge_Norm_Q_to_Obs(i,j,Num_Obs) = 1;                     % �浹 ���ɼ��� �����Ƿ� �浹 on = 1
                                     else
                                        Judge_Norm_Q_to_Obs(i,j,Num_Obs) = 0;    
                                     end
                                 
                                     if and(parameter_t{i,j,Num_Obs} >= 0 , parameter_t{i,j,Num_Obs} <= 1)  % �Ķ���Ͱ� t�� 0~1���̿� �����ؾ� ��ֹ��� �̵���λ� ����
                                         Judge_parameter_t(i,j,Num_Obs) = 1;                                % ���� 0~1���̿� ������ �浹 on = 1
                                     else
                                        Judge_parameter_t(i,j,Num_Obs) = 0;
                                     end
                                 end
                             end
                        end
                        

                        % �Ǻ���� �� ���� ������ ��� 1�� ������ ��ֹ��� �̵���ο� �����Ƿ� �Ǻ������ ����
                        % �̿��� �Ѵ� �����ϴ��� Ȯ��.
                        for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                    Judge_Matrix(i,j,Num_Obs) = (Judge_Norm_Q_to_Obs(i,j,Num_Obs)) .* (Judge_parameter_t(i,j,Num_Obs));

                                 end
                             end
                        end
                                 
                                 for Num_Obs = 1 : length(Obs_Circle)                                
                                    if max(max((Judge_Matrix(:,:,Num_Obs)))) == 1
                                        Switch_F_Rep(Num_Obs) = 1;
                                    else
                                        Switch_F_Rep(Num_Obs) = 0;
                                    end
                                 end


%% ��ֹ��� ������ġ�� �Ÿ� ����

    for i = 1 : DOF
           for j = 1 : length(mid_point)
               for Num_Obs = 1 : length(Obs_Circle)
                   D_Vec{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;
                   D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs); % �� �״�� �Ÿ� (������ġ-�߽�) - ������    
              % D_Norm �Ǻ�
                    if D_Norm{i,j,Num_Obs} <= Bound_con_2(Num_Obs)   % ���2���� ������ ��ֹ� ��ι��� �ִٰ� �Ǵ�
                        Judge_D_Norm{i,j,Num_Obs} = 1;
                    else
                        Judge_D_Norm{i,j,Num_Obs} = 0;
                    end
               end
           end
    end
% DNORM=cell2mat(Judge_D_Norm)

%% ������ġ�� ��ֹ� �Ÿ� * ��ֹ��� ��ο� �ɸ����� Ȯ��

% ������ ��ֹ��� ��λ� �����ϴ��� Ȯ���ϴ� Switch_F_Rep��
% ��ֹ��� �Ÿ��� �����ؼ� �Ÿ��� ��谪���� ������ ����ġ On �ϴ� Judge_D_Norm�� ��������
% ��λ� �����ϸ鼭 ������ġ�� ��ֹ��� ����� ����� ��쿡�� On�� ����� ���� ����

    for i = 1 : DOF
           for j = 1 : length(mid_point)
               for Num_Obs = 1 : length(Obs_Circle)
                    Both_Judge{i,j,Num_Obs} = Judge_D_Norm{i,j,Num_Obs}*Switch_F_Rep(Num_Obs);
               end
           end
    end

%% Switch_Obs �� ����
% ������ Both_Judge�� 1�̸� ����ġ On ������ ���� ��Ľ��̹Ƿ�
% ��Ľ��� �� �ϳ��� 1�̶�� �� ��ֹ��� �������� �Ѵ� ����
% Max�� �̿��� �ִ밪�� 0�̸� ��ֹ��� ���� ����


             for Num_Obs = 1 : length(Obs_Circle)                                
                if max(max(cell2mat(Both_Judge(:,:,Num_Obs)))) == 1
                    Switch_Obs(Num_Obs) = 1;
                else
                    Switch_Obs(Num_Obs) = 0;
                end
             end
             

% C_d=cell2mat(Norm_Q_to_Obs)
% C_Jd=cell2mat(Judge_Norm_Q_to_Obs)
% C_t=cell2mat(parameter_t)
Norm_Q_to_Obs
C_JM=[Judge_Norm_Q_to_Obs Judge_parameter_t Judge_Matrix]
% C_SwFRe=Switch_F_Rep
% D_Norm
% Switch_F_Rep
% Judge_D_Norm
% Both_Judge
% Switch_Obs
 
Proj_parameter_t
parameter_t % 



end

