function [ Norm_Q_to_Obs, Switch_F_Rep ] = So_Distance_Att_Obs( Goal_All, P_All )
%SO_DISTANCE_ATT_OBS Summary of this function goes here
%   Detailed explanation goes here
%   로봇암의 현재 위치 -> 목표위치의 벡터와 
%   장애물 Obs사이의 최단 거리
%   이를 구하는 이유는 로봇암의 움직이는 경로에 장애물이 존재하는가 판별
%   최단거리 d가 장애물의 반지름 r보다 작다면 로봇암의 이동경로에 장애물이 충돌
%   Norm_Q_to_Obs : 현재위치와 목표점 벡터와 장애물 수선의 발 내렸을 때의 길이 최단거리 d
%   Switch_F_Rep  : 척력스위치 1이면 척력 On, 0이면 off. 예를들어 장애물이 2개면 [1 0] 로 표시 이경우
%   첫번째 장애물은 On, 두번째는 Off

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF

            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)
% i= DOF
% j=length(mid_point)
% Num_Obs = 1
                            Vec_P_to_Goal{i,j} =  Goal_All{i,j}-P_All{i,j};        %현재 로봇의 위치 P 에서 목표점 G까지
                            Vec_P_to_Obs{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-P_All{i,j}; %현재 로봇의 위치 P 에서 장애물 Obs까지
                           
                            %정사영 구조체
                            Proj_d{i,j} =    ( ( dot(Vec_P_to_Goal{i,j},Vec_P_to_Obs{i,j,Num_Obs}) )  / norm(Vec_P_to_Goal{i,j}) ) .* ( Vec_P_to_Goal{i,j}./norm(Vec_P_to_Goal{i,j}) );
                            Vec_Q_to_Obs{i,j,Num_Obs} =  Vec_P_to_Obs{i,j,Num_Obs} - Proj_d{i,j}; % 정사영에서 내린 수선의발 Q와 장애물의 벡터
                            Norm_Q_to_Obs{i,j,Num_Obs} = norm(Vec_Q_to_Obs{i,j,Num_Obs});         % 수선의발 QO의 거리
                            
                        end
                    end
            end
            

                    %% 매개변수 t 찾기 (t가 음수면 로봇암보다 뒤에있다는 뜻)
            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)

                            T_Vec_P_to_Goal{i,j} =  Goal_All{i,j}-P_All{i,j};        %현재 로봇의 위치 P 에서 목표점 G까지
                            T_Vec_P_to_Obs{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-P_All{i,j}; %현재 로봇의 위치 P 에서 장애물 Obs까지
                           
                            parameter_t{i,j,Num_Obs} = (  ( P_All{i,j}(1)-Obs_Circle{Num_Obs}(1) )*(P_All{i,j}(1)-Goal_All{i,j}(1)) + ( P_All{i,j}(2)-Obs_Circle{Num_Obs}(2) )*(P_All{i,j}(2)-Goal_All{i,j}(2)) + ( P_All{i,j}(3)-Obs_Circle{Num_Obs}(3) )*(P_All{i,j}(3)-Goal_All{i,j}(3))  ) ./ norm(T_Vec_P_to_Goal{i,j}).^2  ;
                        end
                    end
            end
            
            
            
%% 장애물이 경로를 방해하냐 안하냐 구분

                        %판별행렬 만들기
                        for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                     if Norm_Q_to_Obs{i,j,Num_Obs} < 1.5*Obs_Cir_Radius(Num_Obs)      % 벡터 Att와 장애물의 최단거리가 장애물 반지름보다 작다면
                                        Judge_Norm_Q_to_Obs{i,j,Num_Obs} = 1;                     % 충돌 가능성이 있으므로 충돌 on = 1
                                     else
                                        Judge_Norm_Q_to_Obs{i,j,Num_Obs} = 0;    
                                     end
                                 
                                     if and(parameter_t{i,j,Num_Obs} >= 0 , parameter_t{i,j,Num_Obs} <= 1)  % 파라미터값 t는 0~1사이에 존재해야 장애물이 이동경로상에 존재
                                         Judge_parameter_t{i,j,Num_Obs} = 1;                                % 따라서 0~1사이에 있으면 충돌 on = 1
                                     else
                                        Judge_parameter_t{i,j,Num_Obs} = 0;
                                     end
                                 end
                             end
                        end
                        

                        % 판별행렬 두 가지 조건이 모두 1일 때에만 장애물이 이동경로에 있으므로 판별행렬의 곱을
                        % 이용해 둘다 만족하는지 확인.
                      
                                 for Num_Obs = 1 : length(Obs_Circle)
                                    Judge_Matrix{:,:,Num_Obs} = cell2mat(Judge_Norm_Q_to_Obs(:,:,Num_Obs)) .* cell2mat(Judge_parameter_t(:,:,Num_Obs));
% % % % %                                     if and(min(min(cell2mat(parameter_t(:,:,Num_Obs)))) < 0, max(max(cell2mat(parameter_t(:,:,Num_Obs)))) > 1)     % 가장 작은 값이 음수면 이 장애물은 로봇팔보다 뒤에 있음
% % % % %                                         Switch_F_Rep(Num_Obs) = 0;                                  % 장애물이 로봇팔보다 뒤에 있으므로 목표점까지 가는데 방해가안됨
% % % % %                                     else
% % % % %                                         if min(min(cell2mat(Norm_Q_to_Obs(:,:,Num_Obs)))) < Obs_Cir_Radius(Num_Obs)       % 최단거리가 각각 정의된 반지름 r 보다 작다면,
% % % % %                                             Switch_F_Rep(Num_Obs) = 1;                     % 장애물이 충돌하므로 척력 필요
% % % % %                                         else
% % % % %                                             Switch_F_Rep(Num_Obs) = 0;                     % 장애물이 로봇팔보다는 가까이 있지만 충돌하지는 않음
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
% %                       parameter_t   % 매개변수
% % Norm_Q_to_Obs
% % cell2mat(Judge_Matrix)
% % 
% % Judge_Norm_Q_to_Obs
% % 
% % Judge_parameter_t

end

