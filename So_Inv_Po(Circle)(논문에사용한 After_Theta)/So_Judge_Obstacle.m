function [ Switch_Obs,Norm_Q_to_Obs, Switch_F_Rep,Judge_Matrix_0,Judge_Matrix ] = So_Judge_Obstacle( Goal_All, P_All )
%SO_DISTANCE_ATT_OBS Summary of this function goes here
%   Detailed explanation goes here
%   로봇암의 현재 위치 -> 목표위치의 벡터와 
%   장애물 Obs사이의 최단 거리
%   이를 구하는 이유는 로봇암의 움직이는 경로에 장애물이 존재하는가 판별
%   최단거리 d가 장애물의 반지름 r보다 작다면 로봇암의 이동경로에 장애물이 충돌

% Switch_Obs : 판별행렬 Both_Judge의 모든 값이 0이면 Off, 아니면 On
%		On(1) : 경로상에 장애물이 존재
%		Off(0) : 경로상에 장애물이 없음  
%   Norm_Q_to_Obs : 현재위치와 목표점 벡터와 장애물 수선의 발 내렸을 때의 길이 최단거리 d
%   Switch_F_Rep  : 척력스위치 1이면 척력 On, 0이면 off. 예를들어 장애물이 2개면 [1 0] 로 표시 이경우
%   첫번째 장애물은 On, 두번째는 Off
% Judge_Matrix_0 : 수선의발의 거리 d가 장애물 반지름보다 작을 때, 매개변수 t가 0~1사이일때 
%  0<t<1, d < r 두 조건을 모두 만족할 때 On
% Judge_Matrix_0 : 수선의발의 거리 d가 장애물 반지름+Boundary 보다 작을 때, 매개변수 t가 0~1사이일때 
% 0<t<1, d < r+Boundary 두 조건을 모두 만족할 때 On

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Bound_con_2

            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)
% i= DOF
% j=length(mid_point)
% Num_Obs = 1
                            Vec_P_to_Goal{i,j} =  Goal_All{i,j}-P_All{i,j};        %현재 로봇의 위치 P 에서 목표점 G까지
                            Vec_P_to_Obs{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-P_All{i,j}; %현재 로봇의 위치 P 에서 장애물 Obs까지
                           
                            %정사영 구조체
                            Proj_d{i,j,Num_Obs} =    ( ( dot(Vec_P_to_Goal{i,j},Vec_P_to_Obs{i,j,Num_Obs}) )  / norm(Vec_P_to_Goal{i,j}) ) .* ( Vec_P_to_Goal{i,j}./norm(Vec_P_to_Goal{i,j}) );
                            Vec_Q_to_Obs{i,j,Num_Obs} =  Vec_P_to_Obs{i,j,Num_Obs} - Proj_d{i,j,Num_Obs}; % 정사영에서 내린 수선의발 Q와 장애물의 벡터
                            Norm_Q_to_Obs{i,j,Num_Obs} = norm(Vec_Q_to_Obs{i,j,Num_Obs});         % 수선의발 QO의 거리
                            
                        end
                    end
            end
            
            %% 부록같은 느낌 아래 매개변수를 t norm(정사영)/norm(PG)벡터 이용해서 만들기
              % t = norm(정사영)/norm(PG) 이용해 만들수 있고 이 값이 0~1사이가 되어야하며
              % 정사영의 벡터가 PG와 다르면 음수이면 이경우는 고려할 필요없이 충돌안일어남
              % 까먹을까봐 다시 적지만 충돌 조건은 매개변수 t가 0~1사이 값이면서
              % 점(장애물)과 직선(현재위치에서 목표위치 벡터)사이의 최단거리 d가 장애물의 반지름r
              % 보다 작은조건에서 충돌할 가능성이 있으므로 이 때에 척력 조건이 On(1)이 된다.
              
              % 어쨋든 아래 매개변수를 구했으나 정사영을 이용해서 구해보자
              
              
              for i = 1 : DOF
                 for j = 1 : length(mid_point)
                        for Num_Obs = 1 : length(Obs_Circle)
                            Proj_parameter_t{i,j,Num_Obs} = norm(Proj_d{i,j,Num_Obs})./ norm(Vec_P_to_Goal{i,j});
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
            
            %파라미터 t의 값은 0~1 사이에 존재해야만 장애물이 경로상에 있다는 소리.
            
%% 0차 장애물이 경로를 방해하냐 안하냐 구분

                        %판별행렬 만들기
                        for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                     if Norm_Q_to_Obs{i,j,Num_Obs} < Obs_Cir_Radius(Num_Obs)      % 벡터 Att와 장애물의 최단거리가 장애물 반지름보다 작다면
                                        Judge_Norm_Q_to_Obs_0(i,j,Num_Obs) = 1;                     % 충돌 가능성이 있으므로 충돌 on = 1
                                     else
                                        Judge_Norm_Q_to_Obs_0(i,j,Num_Obs) = 0;    
                                     end
                                 
                                     if and(parameter_t{i,j,Num_Obs} >= 0 , parameter_t{i,j,Num_Obs} <= 1)  % 파라미터값 t는 0~1사이에 존재해야 장애물이 이동경로상에 존재
                                         Judge_parameter_t_0(i,j,Num_Obs) = 1;                                % 따라서 0~1사이에 있으면 충돌 on = 1
                                     else
                                        Judge_parameter_t_0(i,j,Num_Obs) = 0;       %0차나 1차나 사실 이건 같으니까 굳이 안바꿔도 될듯.
                                     end
                                 end
                             end
                        end
                        

                        % 판별행렬 두 가지 조건이 모두 1일 때에만 장애물이 이동경로에 있으므로 판별행렬의 곱을
                        % 이용해 둘다 만족하는지 확인.
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

 %% 목표위치와 장애물 사이의 길이,그리고 경계값

% 위에서는 현재 위치와 장애물 경로를 따졌다면,
% 여기서는 마지막 위치인 목표위치와 장애물의 거리를 판단한다.
% 장애물과 목표위치가 아주 근접하다면, 장애물의 경계값안에 목표위치가 존재하게 되고
% 이는 스위치가 항상 On이 될 수 밖에 없으므로 가상의 인력 척력 해제 조건이 스위치가 Off 인데
% 이럴 경우 스위치가 꺼지지 않는 경우가 존재한다.
% 이게 정답은 아니지만 일단 장애물 경계값2가 1.5r 이므로 최소거리가 1.5r보다는 커야한다.



            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                    for Num_Obs = 1 : length(Obs_Circle)
                        Obs_Goal{i,j,Num_Obs} =   Obs_Circle{Num_Obs}'-Goal_All{i,j}; %현재 로봇의 위치 P 에서 장애물 Obs까지
                        Norm_Obs_Goal(i,j,Num_Obs) = norm(Obs_Goal{i,j,Num_Obs});
                    
% % %                             % 판단행렬 만들기
% % %                             % 2차경계선보다 작으면 0, 아니면 1? 일단은
% % %                             if Norm_Obs_Goal(i,j,Num_Obs) < 1.5*Obs_Cir_Radius(Num_Obs)
% % %                                 Judge_Norm_Obs_Goal(i,j,Num_Obs) = 0;
% % %                             else
% % %                                 Judge_Norm_Obs_Goal(i,j,Num_Obs) = 1;
% % %                             end       
                    
                    end
                 end
            end
%             Judge_Norm_Obs_Goal
            
            
            % 로봇 위치와 장애물 가장 가까운 거리
            for Num_Obs = 1 : length(Obs_Circle)
                Minimum_Dis_G_Obs(Num_Obs) = min(min(Norm_Obs_Goal(:,:,Num_Obs)));
            end
%           Minimum_Dis_G_Obs
            % 경계 조건하고 최소거리 비교
            for Num_Obs = 1 : length(Obs_Circle)
                if Switch_F_Rep_0(Num_Obs)  == 1        % 장애물이 경로상에 존재하면서 목표위치가 근접하면
                                              % 1.5r 보다 목표위치가 작을 수 있으므로
                                              % 목표위치는 가장 가까운 거리만큼으로 줄임
                    if Minimum_Dis_G_Obs(Num_Obs) < Bound_con_2(Num_Obs)    
                        disp('경계값변경')
                        Bound_con_2(Num_Obs) = Minimum_Dis_G_Obs(Num_Obs)-0.2
                    end
                end
            end               
                            
                                 
                                 
%% 2차 Boundary가 장애물이 경로를 방해하냐 안하냐 구분

                        %판별행렬 만들기
                        for i = 1 : DOF
                             for j = 1 : length(mid_point)
                                 for Num_Obs = 1 : length(Obs_Circle)
                                     if Norm_Q_to_Obs{i,j,Num_Obs} < (Bound_con_2(Num_Obs)+Obs_Cir_Radius(Num_Obs))      % 벡터 Att와 장애물의 최단거리가 장애물 반지름보다 작다면
                                        Judge_Norm_Q_to_Obs(i,j,Num_Obs) = 1;                     % 충돌 가능성이 있으므로 충돌 on = 1
                                     else
                                        Judge_Norm_Q_to_Obs(i,j,Num_Obs) = 0;    
                                     end
                                 
                                     if and(parameter_t{i,j,Num_Obs} >= 0 , parameter_t{i,j,Num_Obs} <= 1)  % 파라미터값 t는 0~1사이에 존재해야 장애물이 이동경로상에 존재
                                         Judge_parameter_t(i,j,Num_Obs) = 1;                                % 따라서 0~1사이에 있으면 충돌 on = 1
                                     else
                                        Judge_parameter_t(i,j,Num_Obs) = 0;
                                     end
                                 end
                             end
                        end
                        

                        % 판별행렬 두 가지 조건이 모두 1일 때에만 장애물이 이동경로에 있으므로 판별행렬의 곱을
                        % 이용해 둘다 만족하는지 확인.
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


%% 장애물과 현재위치의 거리 측정

    for i = 1 : DOF
           for j = 1 : length(mid_point)
               for Num_Obs = 1 : length(Obs_Circle)
                   D_Vec{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;
                   D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs); % 말 그대로 거리 (현재위치-중심) - 반지름    
              % D_Norm 판별
                    if D_Norm{i,j,Num_Obs} <= Bound_con_2(Num_Obs)   % 경계2보다 작으면 장애물 경로방해 있다고 판단
                        Judge_D_Norm{i,j,Num_Obs} = 1;
                    else
                        Judge_D_Norm{i,j,Num_Obs} = 0;
                    end
               end
           end
    end
% DNORM=cell2mat(Judge_D_Norm)

%% 현재위치와 장애물 거리 * 장애물이 경로에 걸리는지 확인

% 위에서 장애물이 경로상에 존재하는지 확인하는 Switch_F_Rep와
% 장애물의 거리를 측정해서 거리가 경계값보다 작으면 스위치 On 하는 Judge_D_Norm을 구했으니
% 경로상에 존재하면서 현재위치가 장애물에 충분히 가까운 경우에만 On을 만들기 위한 곱셈

    for i = 1 : DOF
           for j = 1 : length(mid_point)
               for Num_Obs = 1 : length(Obs_Circle)
                    Both_Judge{i,j,Num_Obs} = Judge_D_Norm{i,j,Num_Obs}*Switch_F_Rep(Num_Obs);
               end
           end
    end

%% Switch_Obs 온 오프
% 위에서 Both_Judge가 1이면 스위치 On 이지만 저건 행렬식이므로
% 행렬식중 단 하나라도 1이라면 그 장애물은 고려해줘야 한다 따라서
% Max를 이용해 최대값이 0이면 장애물은 무시 가능


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

