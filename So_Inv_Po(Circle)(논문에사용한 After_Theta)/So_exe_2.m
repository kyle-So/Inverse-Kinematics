clc
clear all
clf
tic
global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius
% Ini_t = [(pi/180)*169.9261 (pi/180)*26.0345 (pi/180)*67.8201 (pi/180)*88.4250 (pi/180)*79.0336 (pi/180)*31.815  (pi/180)*(-15.5221)];     

% Ini_t = [(pi/180)*(-3.074) (pi/180)*88.1191 (pi/180)*95.1007 (pi/180)*96.1578 (pi/180)*92.4253 (pi/180)*85.134  (pi/180)*86.7295];
% Goal_t = [(pi/180)*169.9261 (pi/180)*26.0345 (pi/180)*67.8201 (pi/180)*88.4250 (pi/180)*79.0336 (pi/180)*31.815  (pi/180)*(-15.5221)];     
%     d = [278 0 425 0 425 0 72];
%     a = [0 0 0 0 0 0 0];
%     alpha = [-90 90 -90 90 -90 90 0];

% UR 초기값 및 결과
%  Ini_t = [(pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90  (pi/180)*90];
 Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0  (pi/180)*0];
% Goal_t = [(pi/180)*(33) (pi/180)*33 (pi/180)*(-41) (pi/180)*76 (pi/180)*(-9) (pi/180)*(-15)  (pi/180)*26];     
Goal_t = [(pi/180)*(-3.6313) (pi/180)*29.3335 (pi/180)*(16.9509) (pi/180)*75.8842 (pi/180)*(45.5186) (pi/180)*(-28.4714)  (pi/180)*15];

    d = [278 0 425 0 425 0 72];
    a = [0 0 0 0 0 0 0];
    alpha = [-90 90 -90 90 -90 90 0];
 History_Ave_Theta(:,1) = Ini_t*(180/pi);  

    
%% DH 및 초기값 정의
% Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 ];
% Goal_t = [(pi/180)*90 (pi/180)*0 (pi/180)*0 ];     
%     d = [0 0 0];
%     a = [3 2 2];
%     alpha = [0 0 0 0];

mid_point = [0.25 0.5 0.75 1];
L_end = length(mid_point);
DOF = length(Ini_t);
DOF*L_end;
Coeff_att = [1]; Coeff_rep = [];
Local_Minimum_Timer =0; Check_Switch_Timer = 0;
Min_Point = [0;0;0]; Divid = 0;
Ex_Switch = 0; Ex_Num_Obs = 1; Ex_Coef_F_rep = 10; Cre_Plot =1; Parameter_Switch = 0;
Ex_Sum_Torque_att = zeros(DOF,1); Ex_Sum_Torque_rep = zeros(DOF,1);

% 가중치
for i = 1 : (DOF*L_end)
    if i == 1 || i == (DOF*L_end)
        Value = 1;
%         Value = 0;
    else
        Value = 50;
%         Value = 0;
    end
    Coeff_rep = [Coeff_rep Value];
end
% Coeff_rep = [1 1 1 1 1 1 1 1];

Eraser = 1;
Theta = Ini_t;
Before_Check = zeros(DOF,L_end);
% 2D일때 장애물
% Obstacle{1} = [3 3 0 ; 3 4 0; 4 4 0; 4 3 0; 3 3 0];       % 1번 장애물
% 
% Obs_Circle{1} = [3 4 0];
% Obs_Circle{2} = [6.5 2 0];
% Obs_Cir_Radius = [1 1];

% 7축일때 장애물
Obs_Circle{1} = [400 0 700];
% Obs_Circle{2} = [6.5 2 0];
Obs_Cir_Radius = [0];

obs_thrs = [5 5 5 5]  ;                         % 장애물 경계값
% Transe_Obs= Obstacle{1}' ;                       % 세로로 좌표 표현
Bound_con = Obs_Cir_Radius*0.8    ;                   % 경계값을 반지름으로 잡음
Bound_con_2 = Obs_Cir_Radius*3 ;
Try_Num = 800;
Limit_Value =1;


 [Ave_P_All,Ave_X,Ave_Y,Ave_Z]=So_Theta_to_Position( Ini_t )   
%% 장애물

% 2D 사각형
% 	for i = 1:length(Obstacle)
% 		obs = Obstacle{i};
% 		patch(Obstacle{i}(:,1),Obstacle{i}(:,2),Obstacle{i}(:,3),'black');  % x,y 에 색 채움
%     end
    
% 3D 구
figure(1)
hold on
for i = 1 : length(Obs_Circle)
[Cir_x,Cir_y,Cir_z] = sphere;               %일단 구 생성
surf((Obs_Cir_Radius(i)*0.8*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*0.8*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*0.8*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)      % 크기 3짜리 5,4,0에 생성
colormap winter
grid off
end
view(45,20)

% figure(2)
% hold on
% for i = 1 : length(Obs_Circle)
% [Cir_x,Cir_y,Cir_z] = sphere;               %일단 구 생성
% surf((Obs_Cir_Radius(i)*0.8*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*0.8*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*0.8*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)      % 크기 3짜리 5,4,0에 생성
% colormap winter
% grid off
% end
%% 좌표계
% hold on
% quiver3(0,0,0,100,0,0, 'color','red', 'LineWidth', 2)
% quiver3(0,0,0,0,100,0, 'color','green', 'LineWidth', 2)
% quiver3(0,0,0,0,0,100, 'color','blue', 'LineWidth', 2)
view(45,20)  % 3D 이미지 보기 좋은 각도
%%  계산
for  Repeat = 1 : Try_Num
    Limit_Value = (1 - 3.*(Repeat/Try_Num) );
    if  Limit_Value < 0
          Limit_Value =0
    else
          Limit_Value
    end    
    
ATT_POWER = 0;
ATT_POWER_Re = 0;
REP_POWER = 0;
REP_POWER_2 = 0;

    %     pause(0.2)
%      Current_Coff_2 = zeros(DOF,1);
    Sum_Torque_att = zeros(DOF,1);
    Sum_Torque_att_Reverse = zeros(DOF,1);
    Sum_Torque_rep = zeros(DOF,1);
    Sum_Torque_rep_2 = zeros(DOF,1);
    Ex_Sum_Torque_att = zeros(DOF,1);
    Ex_Sum_Torque_rep = zeros(DOF,1);
    [ P_All,X,Y,Z]=So_Theta_to_Position( Theta )  ;           %초기값이자 변하는 세타
    [ Goal_All,Goal_X,Goal_Y,Goal_Z]=So_Theta_to_Position( Goal_t );
    J=So_Jacobian( Theta );
    History_X(:,Repeat) = cell2mat(X(:,L_end));
    History_Y(:,Repeat) = cell2mat(Y(:,L_end));
    History_Z(:,Repeat) = cell2mat(Z(:,L_end));
    History_Theta(:,Repeat) = Theta*(180/pi);
    Mat_X = cell2mat(X);
    Mat_Y = cell2mat(Y);
    Mat_Z = cell2mat(Z);
    
    %평균Theta 만들기
    
    if Repeat >= 2
        Ave_Theta=(Pre_Theta+Theta)./2 ;
        [Ave_P_All,Ave_X,Ave_Y,Ave_Z]=So_Theta_to_Position( Ave_Theta )  ; 
        History_Ave_Theta(:,Repeat) = Ave_Theta*(180/pi);
    end
    
    
    
    % Att 힘
    for i = 1 : DOF
       for j = 1 : length(mid_point)
            F_att{i,j}= Coeff_att*(Goal_All{i,j}-P_All{i,j}) ;       % 2축이라 Z는 제거함
            Line_att{i,j} = (Goal_All{i,j}-P_All{i,j})/norm((Goal_All{i,j}-P_All{i,j}));
            
            if ATT_POWER < norm(F_att{i,j})
                ATT_POWER = norm(F_att{i,j});
            end
       end
    end
    
    % Rep 힘
    for i = 1 : DOF
           for j = 1 : length(mid_point)
%                F_rep{i,j}=[0,0,0];
               D_Vec{i,j}=[0;0;0];
               Temp_F_rep =[0;0;0];
               for Num_Obs = 1 : length(Obs_Circle)

                       
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
                                Temp_F_rep = Temp_F_rep + 0*Coeff_rep(i+j-1)*( (1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) ) ).*(1/D_Norm{i,j,Num_Obs}).^3 .* (D_Vec{i,j,Num_Obs});
                                
                            else
                                Part_F_rep{i,j,Num_Obs} = [0;0;0];
                                Temp_F_rep = Temp_F_rep + [0;0;0];        % Temp_F의 의미는 장애물이 2개 이상있을때, 두 장애물 모두고려한 반력의 합
                            end
                            F_rep{i,j}=  0*Temp_F_rep;
                            
                            if REP_POWER < norm(F_rep{i,j})
                                REP_POWER = norm(F_rep{i,j});
                            end
                            
                            
               end
           end
    end

    
        % 2차 방어선 Rep 힘  ATT 이용해만든값
    for i = 1 : DOF
           for j = 1 : length(mid_point)
%                F_rep{i,j}=[0,0,0];
               D_Vec_2{i,j}=[0;0;0];
               Temp_F_rep_2 =[0;0;0];
               for Num_Obs = 1 : length(Obs_Circle)

                       
                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                   % 장애물을 사각형으로 생각해서 만든 2D 장애물 코드
                                   %                        Current_Dist_obs = [];
%                                                           Min_Dist_obs = [];
                            % % % % % % % % %         for ob_i = 1 : length( Transe_Obs-1)
                            % % % % % % % % %             Current_Dist_obs =   P_All{i,j}-Transe_Obs(:,ob_i); 
                            % % % % % % % % %             
                            % % % % % % % % %             if ob_i == 1 || (norm(Current_Dist_obs) < norm(Min_Dist_obs))
                            % % % % % % % % %                 D_Vec_2{i,j}= Current_Dist_obs;  % 장애물과 현재의 거리를 벡터값 xyz로 나타냄
                            % % % % % % % % %                 Min_Dist_obs = Current_Dist_obs;
                            % % % % % % % % %                 Min_obs_position{i,j} = Transe_Obs(:,ob_i);
                            % % % % % % % % %             end
                            % % % % % % % % %             
                            % % % % % % % % %         end
                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       

                             % 원일 때 거리 측정.
                         for Cir_i = 1 : DOF
                            for Cir_j =1 : length(mid_point)
                                D_Vec_2{Cir_i,Cir_j,Num_Obs} = P_All{Cir_i,Cir_j}-Obs_Circle{Num_Obs}' ;
                            end
                         end
                         
                         Ch_REP_VEC{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
                         
                         Line_rep{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs}/norm(D_Vec_2{i,j,Num_Obs});

                         
                        D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % 말 그대로 거리 (현재위치-중심) - 반지름
                        Bound_Circle(Num_Obs) = Bound_con(Num_Obs) + Obs_Cir_Radius(Num_Obs) ;             % 경계선1
                        Bound_Circle_2(Num_Obs) = Bound_con_2(Num_Obs) + Obs_Cir_Radius(Num_Obs);          % 경계선2
%                         D_Coff_2_Norm{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs}) 
%                         Distance_Coff_2(i,j) = D_Coff_2_Norm{i,j,Num_Obs}/Bound_con_2
                        
%                           D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs});

                                Line_Theta{i,j,Num_Obs} = acos(dot(Line_att{i,j},Line_rep{i,j,Num_Obs})/(norm(Line_att{i,j})*norm(Line_rep{i,j,Num_Obs}))) *(180/pi);
                        
                        
                            if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
                                
                                
                                
                                if D_Norm_2{i,j,Num_Obs} < Bound_con(Num_Obs)
                                    Current_Coff_2{i,j,Num_Obs} = 0.5;
                                else
                                    Current_Coff_2{i,j,Num_Obs} = (( Bound_con_2(Num_Obs)-D_Norm_2{i,j,Num_Obs} )/( Bound_con_2(Num_Obs)-Bound_con(Num_Obs) ))/2;  %경계선 2보다 작아지면 선형적으로 증가 0~1 까지. 
                                end                                                                                                                  % 1보다 크다는소리는 경계1보다 더 작은값의미
                                Part_F_rep_2{i,j,Num_Obs} = 1.* norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
                                Temp_F_rep_2 = Temp_F_rep_2 + 10.*norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
                            else
                                Current_Coff_2{i,j,Num_Obs} = [0];
                                Part_F_rep_2{i,j,Num_Obs} = [0;0;0];
                                Temp_F_rep_2 = Temp_F_rep_2 + [0;0;0];        % Temp_F의 의미는 장애물이 2개 이상있을때, 두 장애물 모두고려한 반력의 합
                            end
                            
                            
                            
                            
                            if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
                                if Line_Theta{i,j,Num_Obs} >= 90 
%                                   F_rep_2{i,j}= 0.5*Current_Coff_2{i,j,Num_Obs}.*F_att{i,j}.*(Temp_F_rep_2/norm(Temp_F_rep_2));
                                    F_rep_2{i,j}= 1*Current_Coff_2{i,j,Num_Obs}*Temp_F_rep_2;
                                end
                            else
                                  F_rep_2{i,j}=  Temp_F_rep_2;
                            end
                            
                            
                            if REP_POWER_2 < norm(F_rep_2{i,j})
                                REP_POWER_2 = norm(F_rep_2{i,j});
                            end
                            
% %                      if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
% %                         if Line_Theta{i,j,Num_Obs} >= 90 
% % %                                   F_rep_2{i,j}= 0.5*Current_Coff_2{i,j,Num_Obs}.*F_att{i,j}.*(Temp_F_rep_2/norm(Temp_F_rep_2));
% %                             F_att_Reverse{i,j}= (1+Current_Coff_2{i,j,Num_Obs}).*F_att{i,j};
% %                         end
% %                     else
% %                           F_att_Reverse{i,j}=  [0;0;0];
% %                      end
% % 
% %                      if ATT_POWER_Re < norm(F_att_Reverse{i,j})
% %                         ATT_POWER_Re = norm(F_att_Reverse{i,j});
% %                      end
                            
                            
               end
           end
    end
    
                                % Att힘 줄이기
            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                     if min(min(cell2mat(D_Norm_2))) <= Bound_con_2(Num_Obs)
                        if Line_Theta{i,j,Num_Obs} >= 90  %두 직선사이의 각도가 90보다 크면 -> 같은방향이 아니라면. 
                            F_att_Reverse{i,j}= 1*(0.2+Current_Coff_2{i,j,Num_Obs}).*F_att{i,j};
                        end
                     else
                          F_att_Reverse{i,j}=  [0;0;0];
                     end

                     if ATT_POWER_Re < norm(F_att_Reverse{i,j})
                        ATT_POWER_Re = norm(F_att_Reverse{i,j});
                     end
                 end
            end
            
%             Diff_CC = cell2mat(Current_Coff_2)- Before_Check
% Before_Check = cell2mat(Current_Coff_2)            
%  norm(Before_Check)
% FF=cell2mat(F_rep_2)
% VOC = cell2mat(D_Vec_2)
% CCCC = cell2mat(Ch_REP_VEC)
%     ATT_POWER
%     ATT_POWER_Re
%     REP_POWER
%     REP_POWER_2
%     PUL_OR_Miu = (1./cell2mat(D_Norm_2)) - ( 1/Bound_con_2(Num_Obs) )
% %             % 2차 방어선 Rep 힘 -> 의미가 없음 바운더리는 커지는데 반력은 반지름 근처가야 커짐
% %     for i = 1 : DOF
% %            for j = 1 : length(mid_point)
% % %                F_rep{i,j}=[0,0,0];
% %                D_Vec_2{i,j}=[0;0;0];
% %                Temp_F_rep_2 =[0;0;0];
% %                for Num_Obs = 1 : length(Obs_Circle)
% % 
% %                        
% %                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %                                    % 장애물을 사각형으로 생각해서 만든 2D 장애물 코드
% %                                    %                        Current_Dist_obs = [];
% % %                                                           Min_Dist_obs = [];
% %                             % % % % % % % % %         for ob_i = 1 : length( Transe_Obs-1)
% %                             % % % % % % % % %             Current_Dist_obs =   P_All{i,j}-Transe_Obs(:,ob_i); 
% %                             % % % % % % % % %             
% %                             % % % % % % % % %             if ob_i == 1 || (norm(Current_Dist_obs) < norm(Min_Dist_obs))
% %                             % % % % % % % % %                 D_Vec_2{i,j}= Current_Dist_obs;  % 장애물과 현재의 거리를 벡터값 xyz로 나타냄
% %                             % % % % % % % % %                 Min_Dist_obs = Current_Dist_obs;
% %                             % % % % % % % % %                 Min_obs_position{i,j} = Transe_Obs(:,ob_i);
% %                             % % % % % % % % %             end
% %                             % % % % % % % % %             
% %                             % % % % % % % % %         end
% %                              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
% % 
% %                              % 원일 때 거리 측정.
% %                          for Cir_i = 1 : DOF
% %                             for Cir_j =1 : length(mid_point)
% %                                 D_Vec_2{Cir_i,Cir_j,Num_Obs} = P_All{Cir_i,Cir_j}-Obs_Circle{Num_Obs}' ;
% %                             end
% %                          end
% %                          
% %                          Line_rep{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs}/norm(D_Vec_2{i,j,Num_Obs});
% % 
% %                          
% %                         D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % 말 그대로 거리 (현재위치-중심) - 반지름
% %                         Bound_Circle(Num_Obs) = Bound_con(Num_Obs) + Obs_Cir_Radius(Num_Obs) ;             % 경계선1
% %                         Bound_Circle_2(Num_Obs) = Bound_con_2(Num_Obs) + Obs_Cir_Radius(Num_Obs);          % 경계선2
% % %                         D_Coff_2_Norm{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs}) 
% % %                         Distance_Coff_2(i,j) = D_Coff_2_Norm{i,j,Num_Obs}/Bound_con_2
% %                         
% % %                           D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs});
% % 
% %                                 Line_Theta{i,j,Num_Obs} = acos(dot(Line_att{i,j},Line_rep{i,j,Num_Obs})/(norm(Line_att{i,j})*norm(Line_rep{i,j,Num_Obs}))) *(180/pi);
% %                         
% % 
% %                             REP_POWER_2 = (1/D_Norm_2{i,j,Num_Obs}).^3
% % 
% %                             if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
% %                                 
% %                                 Current_Coff_2{i,j,Num_Obs} = ( Bound_con_2(Num_Obs)-D_Norm_2{i,j,Num_Obs} )/( Bound_con_2(Num_Obs)-Bound_con(Num_Obs) );  %경계선 2보다 작아지면 선형적으로 증가 0~1 까지. 
% %                                                                                                                                                         % 1보다 크다는소리는 경계1보다 더 작은값의미
% %                                 
% %                                 Part_F_rep_2{i,j,Num_Obs} = Coeff_rep(i+j-1)*( (1/D_Norm_2{i,j,Num_Obs}) - ( 1/Bound_con_2(Num_Obs) ) ).*(1/D_Norm_2{i,j,Num_Obs}).^3 .* (D_Vec_2{i,j,Num_Obs});
% %                                 Temp_F_rep_2 = Temp_F_rep_2 + 1000000*Coeff_rep(i+j-1)*( (1/D_Norm_2{i,j,Num_Obs}) - ( 1/Bound_con_2(Num_Obs) ) ).*(1/D_Norm_2{i,j,Num_Obs}).^3 .* (D_Vec_2{i,j,Num_Obs});
% %                             else
% %                                 Part_F_rep_2{i,j,Num_Obs} = [0;0;0];
% %                                 Temp_F_rep_2 = Temp_F_rep_2 + [0;0;0];        % Temp_F의 의미는 장애물이 2개 이상있을때, 두 장애물 모두고려한 반력의 합
% %                             end
% %                             
% %                             if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
% %                                 if Line_Theta{i,j,Num_Obs} >= 90 
% % %                                   F_rep_2{i,j}= 0.5*Current_Coff_2{i,j,Num_Obs}.*F_att{i,j}.*(Temp_F_rep_2/norm(Temp_F_rep_2));
% %                                     F_rep_2{i,j}= 0.5*Current_Coff_2{i,j,Num_Obs}*Temp_F_rep_2;
% %                                 end
% %                             else
% %                                   F_rep_2{i,j}=  Temp_F_rep_2;
% %                             end
% %                             
% %                end
% %            end
% %     end
    
    
%     Line_Theta
%     Check_Last1 = D_Norm_2{1,1,Num_Obs};
% Check_Last2 = D_Norm_2{2,1,Num_Obs};
% Check_Last3 = D_Norm_2{3,1,Num_Obs};
% Check_Last4 = D_Norm_2{4,1,Num_Obs};
% Check_Last5 = D_Norm_2{5,1,Num_Obs};
% Check_Last6 = D_Norm_2{6,1,Num_Obs};
% Check_Last7 = D_Norm_2{7,1,Num_Obs};
% DN= D_Norm_2
% BC=Bound_con_2(Num_Obs)
% if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
% FF1 =Current_Coff_2{1,1,1};
% FF2 = Current_Coff_2{2,1,1};
% FF3 = Current_Coff_2{3,1,1};
% FF4 = Current_Coff_2{4,1,1};
% FF5 = Current_Coff_2{5,1,1};
% FF6 = Current_Coff_2{6,1,1};
% FF7 = Current_Coff_2{7,1,1};
%  Ohter = [FF1;FF2;FF3;FF4;FF5;FF6;FF7]
% end
% Check_Full = [Check_Last1;Check_Last2;Check_Last3;Check_Last4;Check_Last5;Check_Last6;Check_Last7]
 
    
    %추가 조건
    %==================================================================================================================================================================================================================
    if Repeat >= 4
        Current_Diff_X = History_X(:,Repeat)-History_X(:,Repeat-1);
        Current_Diff_Y = History_Y(:,Repeat)-History_Y(:,Repeat-1);
        Current_Diff_Z = History_Z(:,Repeat)-History_Z(:,Repeat-1);  
        
        Pre_Current_Diff_X = History_X(:,Repeat-1)-History_X(:,Repeat-2);
        Pre_Current_Diff_Y = History_Y(:,Repeat-1)-History_Y(:,Repeat-2);
        Pre_Current_Diff_Z = History_Z(:,Repeat-1)-History_Z(:,Repeat-2); 
        
        Diff_Two_X = History_X(:,Repeat)-History_X(:,Repeat-2);
        Diff_Two_Y = History_Y(:,Repeat)-History_Y(:,Repeat-2);
        Diff_Two_Z = History_Z(:,Repeat)-History_Z(:,Repeat-2); 
        
%         Pre_Diff_Two_X = History_X(:,Repeat-1)-History_X(:,Repeat-3);
%         Pre_Diff_Two_Y = History_Y(:,Repeat-1)-History_Y(:,Repeat-3);
%         Pre_Diff_Two_Z = History_Z(:,Repeat-1)-History_Z(:,Repeat-3); 
        
        Current_Diff_XYZ = [Current_Diff_X;Current_Diff_Y;Current_Diff_Z];
        Pre_Current_Diff_XYZ = [Pre_Current_Diff_X;Pre_Current_Diff_Y;Pre_Current_Diff_Z];
        Diff_Two_XYZ = [Diff_Two_X;Diff_Two_Y;Diff_Two_Z];
%         Pre_Diff_Two_XYZ = [Pre_Diff_Two_X;Pre_Diff_Two_Y;Pre_Diff_Two_Z];

        [Current_Diff_XYZ Pre_Current_Diff_XYZ];  % 이거 곱해서 음수가 나오면 부호가 바뀌는거
%         [Pre_Diff_Two_XYZ Diff_Two_XYZ] %결과적으로 이 두값은 거의 같다 빼기 조건으로 못씀
        % 변화량의 부호가 변화고, -2의 차가 작은경우를 조건으로 쓰면?
        
        
        % 로컬미니멈 위치 찾기
        if Current_Diff_XYZ.*Pre_Current_Diff_XYZ <= 0
            Check_Switch_Timer = Check_Switch_Timer+5;
        else
            Check_Switch_Timer = 0;
        end
        if Check_Switch_Timer > 100
            Diff_Two_XYZ;  %그냥 궁금해서 표시
                if Diff_Two_XYZ < 0.01
                    Local_Minimum_Timer = Local_Minimum_Timer+5
                    
                    if and(Local_Minimum_Timer >= 80, Local_Minimum_Timer <= 100)
                        Divid = Divid +1;
                        Min_Point = Min_Point + [Mat_X(DOF,L_end);Mat_Y(DOF,L_end);Mat_Z(DOF,L_end)];
                        Average_LM=Min_Point/Divid;
                    end
                    
                end
        end
    end
    %여기까지 계산하고 Local_Minimum_Timer이 100이 넘으면 로컬미니멈에 빠졌다고 할 수 있다.
    
        % 로컬미니멈일때 각종 스위치 ON,OFF 기능
        if Local_Minimum_Timer == 100
            Parameter_Switch = 1;   
        end
    
    
        %로컬 미니멈 100 넘을때 반력 생성
    if and(Parameter_Switch == 1,Cre_Plot==1)
        [Ex_RepX,Ex_RepY,Ex_RepZ] = sphere;               %일단 구 생성
        surf( (Ex_RepX)+Average_LM(1) , (Ex_RepY)+Average_LM(2), (Ex_RepZ)+Average_LM(3),'FaceAlpha',0.2,'EdgeAlpha',0)      % 크기 3짜리 5,4,0에 생성
    %     colormap winter
        grid off
        Cre_Plot = 0;  % 혹시 모를 반복에대비한 또 다른 안전장치
        Ex_Switch = 1;       %로컬미니멈 100되는순간 Ex포텐셜 스위치 ON
        Ex_Obs_Circle{Ex_Num_Obs} = [Average_LM(1) Average_LM(2) Average_LM(3)] 
        Ex_Num_Obs = Ex_Num_Obs+1;
        Ex_Bound_con = 1;
        Ex_Att_Progress_Timer=0;    % 추가반력 인력을 동시 생성하면 반력벡터문제가 발생할 수 있으므로 인력을 먼저 대입하기위한 타이머 초기화
    end
    
    
    % 추가 Att 힘
    if Ex_Switch == 1
%         Local_Minimum_Timer = 0;   % Ex포텐셜 스위치 ON되는순간 로컬미니멈 0 (다음 로컬미니멈 계산을 위한 초기화)
        for i = 1 : DOF
           for j = 1 : length(mid_point)
                Ex_F_att{i,j}= 10*Coeff_att*(-P_All{i,j}) ;       % 2축이라 Z는 제거함
                              % Ex_Att 힘-> 토크
                           Ex_Torque_att{i,j} = J{i,j}'*Ex_F_att{i,j};
                           Ex_Sum_Torque_att = Ex_Sum_Torque_att+Ex_Torque_att{i,j};
           end
        end
       
%         Ex_F_att{i,j}
        Ex_Att_Progress_Timer = Ex_Att_Progress_Timer + 1
        
        if Ex_Att_Progress_Timer >= 50
            for i = 1 : DOF
               for j = 1 : length(mid_point)
    %                Ex_F_rep{i,j}=[0,0,0];
                   Ex_D_Vec{i,j}=[0;0;0];
                   Ex_Temp_F_rep =[0;0;0];
                   for Ex_Num_Obs = 1 : length(Ex_Obs_Circle)

                                 % 원일 때 거리 측정.
                             for Cir_i = 1 : DOF
                                for Cir_j =1 : length(mid_point)
                                    Ex_D_Vec{Cir_i,Cir_j,Ex_Num_Obs} = P_All{Cir_i,Cir_j}-Ex_Obs_Circle{Ex_Num_Obs}' ;
                                end
                             end


                            Ex_D_Norm{i,j,Ex_Num_Obs} = norm(Ex_D_Vec{i,j,Ex_Num_Obs});       % 말 그대로 거리 (현재위치-중심)
    %                           Ex_D_Norm{i,j,Ex_Num_Obs} = norm(Ex_D_Vec{i,j,Ex_Num_Obs});
                                if Ex_D_Norm{i,j,Ex_Num_Obs} <= Bound_con(Ex_Num_Obs)
                                    Ex_Part_F_rep{i,j,Ex_Num_Obs} = Ex_Coef_F_rep*( (1/Ex_D_Norm{i,j,Ex_Num_Obs}) - ( 1/Ex_Bound_con(Ex_Num_Obs) ) ).*(1/Ex_D_Norm{i,j,Ex_Num_Obs}).^3 .* (Ex_D_Vec{i,j,Ex_Num_Obs});
                                    Ex_Temp_F_rep = Ex_Temp_F_rep + Ex_Coef_F_rep*( (1/Ex_D_Norm{i,j,Ex_Num_Obs}) - ( 1/Ex_Bound_con(Ex_Num_Obs) ) ).*(1/Ex_D_Norm{i,j,Ex_Num_Obs}).^3 .* (Ex_D_Vec{i,j,Ex_Num_Obs});
                                else
                                    Ex_Part_F_rep{i,j,Ex_Num_Obs} = [0;0;0];
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
        
    end
    
    % 로컬미니멈 벗어나서 Ex_Att 값 초기화
    if Ex_Switch == 1
        for Num_Obs = 1 : length(Obs_Circle)
            for i = 1 : DOF
                for j =1 : length(mid_point)
                     D_Vec{i,j}=[0;0;0];
                     D_Vec{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;
                     D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % 말 그대로 거리 (현재위치-중심) - 반지름
                end
            end
        end
        
    
        if length(Obs_Circle)> length(Ex_Obs_Circle)
            for Add_for_Ex = (length(Obs_Circle)-length(Ex_Obs_Circle)+1) : length(Obs_Circle)    %Ex 장애물 개수가 더 적으니까 가상의 장애물값을 입력
                                                                                                %예를들어 장애물2개, 로컬미니멈가상 장애물이1개면 로컬쪽 D놈을 2개로 맞춰줌
                for i = 1 : DOF
                  for j =1 : length(mid_point)
                     Ex_D_Norm{i,j,Add_for_Ex} = 10000;       %밑에서 경계값보다 큰 경우 추가 Att,Rep 힘을 0으로 바꾸는데 에러가 나지않기 위한 조건
                  end
                end
            end
        elseif length(Obs_Circle) < length(Ex_Obs_Circle)
            for Add_for_Obs = (length(Ex_Obs_Circle)-length(Obs_Circle)+1) : length(Obs_Circle)    %이거는 반대. 원래장애물 개수가 더 적으니까 가상장애물 개수만큼 맞춰줌
                for i = 1 : DOF
                  for j =1 : length(mid_point)
                     D_Norm{i,j,Add_for_Obs} = 10000;       %밑에서 경계값보다 큰 경우 추가 Att,Rep 힘을 0으로 바꾸는데 에러가 나지않기 위한 조건
                  end
                end
            end            
        end

        
        
        % 수정이 필요한 조건
        if Ex_Att_Progress_Timer >= 51
            if and(D_Norm{DOF,L_end,Num_Obs} > Bound_con(Num_Obs) , Ex_D_Norm{DOF,L_end,Num_Obs} > 1)  %시작장애
               for i = 1 : DOF
                     for j = 1 : length(mid_point)
                           Ex_F_att{i,j}= [0;0;0] ;       % 2축이라 Z는 제거함
                                          % Ex_Att 힘-> 토크
                           Ex_Torque_att{i,j} = zeros(DOF,1);
                           Ex_Sum_Torque_att = zeros(DOF,1);
                     end
               end 
                       Ex_Switch = 0
            end
        end

    end


    
    
    
    %==================================================================================================================================================================================================================
    
    

    
    
    
        % 힘-> 토크
    for i = 1 : DOF
       for j = 1 : length(mid_point)
               % Att 힘-> 토크
           Torque_att{i,j} = J{i,j}'*(F_att{i,j});
           Sum_Torque_att = Sum_Torque_att+Torque_att{i,j};
           Torque_att_Reverse{i,j} = J{i,j}'*(F_att_Reverse{i,j})*Limit_Value;
           Sum_Torque_att_Reverse = Sum_Torque_att_Reverse+Torque_att_Reverse{i,j};
               % Rep 힘-> 토크
                              
           
           
%            if Repeat >= 300
%                Torque_rep{i,j} = J{i,j}'*F_rep{i,j};
%                Sum_Torque_rep = ( Sum_Torque_rep+Torque_rep{i,j}*(800-Repeat))/500;
%                Torque_rep_2{i,j} = J{i,j}'*F_rep_2{i,j};
%                Sum_Torque_rep_2 =( Sum_Torque_rep_2+Torque_rep_2{i,j}*(800-Repeat))/500;            
%            else
               Torque_rep{i,j} = J{i,j}'*F_rep{i,j}*Limit_Value;
               Sum_Torque_rep = Sum_Torque_rep+Torque_rep{i,j};
               Torque_rep_2{i,j} = J{i,j}'*F_rep_2{i,j}*Limit_Value;
               Sum_Torque_rep_2 = Sum_Torque_rep_2+Torque_rep_2{i,j};           
%            end
       end
    end
    
% D_Norm
% F_rep{2,2}

    % 비교를 위해 att만
    Sum_Torque_att;
    Sum_Torque_rep;
    % Theta = Theta + Sum_Torque_att'*0.001


    if Ex_Switch == 0
%         Sum_Torque = Sum_Torque_att+Sum_Torque_rep;
%         Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2;
%         Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2-Sum_Torque_att_Reverse;
Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2-Sum_Torque_att_Reverse;
        % Check_T([1 2 3 4],Repeat) = [Sum_Torque;norm(Sum_Torque)];
        % Check_T_Common([1 2 3],Repeat) = Sum_Torque;
        % Check_T_Norm([1 2 3],Repeat) = Sum_Torque/norm(Sum_Torque);
    elseif Ex_Switch == 1;

%          Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Ex_Sum_Torque_att+Ex_Sum_Torque_rep;
%          Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2+Ex_Sum_Torque_att+Ex_Sum_Torque_rep;
         Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2+Ex_Sum_Torque_att+Ex_Sum_Torque_rep-Sum_Torque_att_Reverse;
      
    end

%     Torque_Check = Sum_Torque'/norm(Sum_Torque)
    Pre_Theta = Theta;
    Theta = Theta + 0.005*(Sum_Torque'/norm(Sum_Torque)) ;
%     Check_Theta = Theta * 180/pi
    % Check_Theta([1 2 3],Repeat) = Theta;

    
    % Att+rep 힘 (시각화 벡터 표현하기 위해 만들었음)
            Weight_F = 0.5;
            if Ex_Switch == 0
                for i = 1 : DOF
                   for j = 1 : length(mid_point)
                       F_Sum{i,j} = (F_att{i,j}+F_rep{i,j})*Weight_F;
                       F_Vatt{i,j} = F_att{i,j}*Weight_F;
                       F_Vrep{i,j} = F_rep{i,j}*Weight_F;
                   end
                end
            elseif Ex_Switch == 1;
                if Ex_Att_Progress_Timer >= 50
                    for i = 1 : DOF
                       for j = 1 : length(mid_point)
                           F_Sum{i,j} = (F_att{i,j}+F_rep{i,j}+Ex_F_att{i,j}+Ex_F_rep{i,j})*Weight_F;
                           F_Vatt{i,j} = (F_att{i,j}+Ex_F_att{i,j})*Weight_F;
                           F_Vrep{i,j} = (F_rep{i,j}+Ex_F_rep{i,j})*Weight_F;
                       end
                    end       
                else
                     for i = 1 : DOF
                       for j = 1 : length(mid_point)
                           F_Sum{i,j} = (F_att{i,j}+F_rep{i,j}+Ex_F_att{i,j})*Weight_F;
                           F_Vatt{i,j} = (F_att{i,j}+Ex_F_att{i,j})*Weight_F;
                           F_Vrep{i,j} = (F_rep{i,j})*Weight_F;
                       end
                     end    
                end
            end


    %% 그래픽 부분

% % % %     figure(1)
% % % %     xlim([-400 400]);
% % % %     ylim([-400 400]);
% % % %     zlim([0 800]);
% % % % %     xlim([-2 10]);
% % % % %     ylim([-2 10]);
% % % % %     zlim([-2 8]);
% % % % %     xlim([4.76 4.96]);
% % % % %     ylim([4.64 4.79]);
% % % % %     zlim([-2 8]);
% % % %     hold on
% % % %     xlabel('x')
% % % %     ylabel('y')
% % % %     zlabel('z')
% % % % 
% % % %     % 마지막 형태 출력
% % % %     for i = 1 : DOF
% % % %            if i == 1
% % % %     %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %둘중하나는
% % % %                plot3([0 Goal_X{i,L_end}],[0 Goal_Y{i,L_end}],[0 Goal_Z{i,L_end}], 'LineWidth', 2,'color','r');     %지워야함 
% % % %            else
% % % %     %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
% % % %                plot3([Goal_X{i-1,L_end} Goal_X{i,L_end}],[Goal_Y{i-1,L_end} Goal_Y{i,L_end}],[Goal_Z{i-1,L_end} Goal_Z{i,L_end}], 'LineWidth', 2,'color','r') ;
% % % %            end
% % % %     end
% % % % 
% % % %     % plot3(Object(1),Object(2),Object(3),'ro')
% % % % 
% % % %     for i = 1 : DOF
% % % %            if i == 1
% % % %     %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %둘중하나는
% % % %                RGraph_L{i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);     %지워야함 
% % % %            else
% % % %     %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
% % % %                RGraph_L{i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
% % % %            end
% % % %     end
% % % %                 Eraser = Eraser +1;
% % % % 
% % % %     for i = 1 : DOF
% % % %     %    for j = 1 : length(mid_point)
% % % %             j = length(mid_point);
% % % % %           Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'c', 'LineWidth', 1);  %경로 표시
% % % % %           Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %벡터표시
% % % % %           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
% % % % %           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
% % % %     %    end
% % % %     end
% % % % 
% % % %     if Repeat ==1
% % % %         pause(3)
% % % %     end
% % % % 
% % % %     pause(0.00002);
% % % % 
% % % %     % % 10개마다 이전 그래프를 지우고 마지막일 경우 마지막 모양은 살림
% % % %     % if Repeat == Try_Num
% % % %     %     for k = 1 : 9
% % % %     %         for i = 1 : DOF
% % % %     %              delete(Graph_L{k,i});
% % % %     %         end 
% % % %     %     end
% % % %     % else
% % % %     %     if Eraser == 11   
% % % %     %          pause(0.02);
% % % %     %         for i = 1 : DOF
% % % %     %              delete(Graph_L{:,i});
% % % %     %         end 
% % % %     %         Eraser = 1;
% % % %     %     end
% % % %     % end 
% % % % 
% % % %     % %     이건 실시간으로 지우는거
% % % %         if Repeat ~= Try_Num 
% % % % %             if mod(Repeat,100) ~= 0
% % % %                 for i = 1 : DOF
% % % %                      delete(RGraph_L{i});
% % % % %                      delete(Point_Dis{i});
% % % %     %                  delete(Disp_Vec{i});
% % % %     %                  delete(Disp_Vatt{i});
% % % %     %                  delete(Disp_Vrep{i});                
% % % %                 end 
% % % % %             end
% % % %         end

   %% 그래픽 부분2(평균값으로 그래픽보여줌)

    figure(1)

    xlim([-600 600]);
    ylim([-600 600]);
    zlim([0 1200]);
%     xlim([-2 10]);
%     ylim([-2 10]);
%     zlim([-2 8]);
%     xlim([4.76 4.96]);
%     ylim([4.64 4.79]);
%     zlim([-2 8]);
    hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')

    % 마지막 형태 출력
    for i = 1 : DOF
           if i == 1
    %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %둘중하나는
               plot3([0 Goal_X{i,L_end}],[0 Goal_Y{i,L_end}],[0 Goal_Z{i,L_end}], 'LineWidth', 2,'color','r');     %지워야함 
           else
    %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
               plot3([Goal_X{i-1,L_end} Goal_X{i,L_end}],[Goal_Y{i-1,L_end} Goal_Y{i,L_end}],[Goal_Z{i-1,L_end} Goal_Z{i,L_end}], 'LineWidth', 2,'color','r') ;
           end
    end

    % plot3(Object(1),Object(2),Object(3),'ro')

    for i = 1 : DOF
           if i == 1
    %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %둘중하나는
               Ave_RGraph_L{i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);     %지워야함 
           else
    %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
               Ave_RGraph_L{i}=plot3([Ave_X{i-1,L_end} Ave_X{i,L_end}],[Ave_Y{i-1,L_end} Ave_Y{i,L_end}],[Ave_Z{i-1,L_end} Ave_Z{i,L_end}], 'LineWidth', 2) ;
           end
    end
                Eraser = Eraser +1;

                
        if mod(Repeat,10) == 1          %10번마다
            for i = 1 : DOF
            %    for j = 1 : length(mid_point)  % 이건 mid_point도 벡터
                    j = length(mid_point);      % 이건 각 링크 끝만 벡터표시
%                   Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'k', 'LineWidth', 1);  %경로 표시
%                   Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %벡터표시
        %           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
        %           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
            %    end
            end
        end
        
                    for i = 1 : DOF
                       for j = 1 : length(mid_point)  % 이건 mid_point도 벡터
%                             j = length(mid_point);  % 이건 각 링크 끝만 벡터표시
        %                   Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'k', 'LineWidth', 1);  %경로 표시
%                           Disp_Vec{i,j}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %벡터표시
%                             Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %벡터표시 각 링크 끝만
                %           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
                %           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
                       end
                     end

    if Repeat ==1
        pause(3)
    end

    pause(0.00002);

    % % 10개마다 이전 그래프를 지우고 마지막일 경우 마지막 모양은 살림
    % if Repeat == Try_Num
    %     for k = 1 : 9
    %         for i = 1 : DOF
    %              delete(Graph_L{k,i});
    %         end 
    %     end
    % else
    %     if Eraser == 11   
    %          pause(0.02);
    %         for i = 1 : DOF
    %              delete(Graph_L{:,i});
    %         end 
    %         Eraser = 1;
    %     end
    % end 

    % %     이건 실시간으로 지우는거
        if Repeat ~= Try_Num 
%             if mod(Repeat,100) ~= 0
            
            for i = 1 : DOF
                     delete(Ave_RGraph_L{i});
                     
%                  if mod(Repeat,10) == 1
%                      delete(Point_Dis{i});
%                      delete(Disp_Vec{i});
    %                  delete(Disp_Vatt{i});
    %                  delete(Disp_Vrep{i});                
%                 end 
            end
            
%             % 벡터값 mid_point 있을때만 실행
%             for i = 1 : DOF
%                 for j = 1 : length(mid_point)
%                     delete(Disp_Vec{i,j});
%                 end
%             end
            
            
%             end
        end        
        
        
end


toc




