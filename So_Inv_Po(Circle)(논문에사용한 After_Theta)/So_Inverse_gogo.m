clc
clear all
clf
tic
global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Bound_con_2 Coeff_rep Try_Num L_end
global Virtual_Bound_con

% Ini_t = [(pi/180)*169.9261 (pi/180)*26.0345 (pi/180)*67.8201 (pi/180)*88.4250 (pi/180)*79.0336 (pi/180)*31.815  (pi/180)*(-15.5221)];     

% Ini_t = [(pi/180)*(-3.074) (pi/180)*88.1191 (pi/180)*95.1007 (pi/180)*96.1578 (pi/180)*92.4253 (pi/180)*85.134  (pi/180)*86.7295];
% Goal_t = [(pi/180)*169.9261 (pi/180)*26.0345 (pi/180)*67.8201 (pi/180)*88.4250 (pi/180)*79.0336 (pi/180)*31.815  (pi/180)*(-15.5221)];     
%     d = [278 0 425 0 425 0 72];
%     a = [0 0 0 0 0 0 0];
%     alpha = [-90 90 -90 90 -90 90 0];

% % % % % UR 초기값 및 결과
% % % % %  Ini_t = [(pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90  (pi/180)*90];
% % % %  Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0  (pi/180)*0];
% % % % Goal_t = [(pi/180)*(33) (pi/180)*33 (pi/180)*(-41) (pi/180)*76 (pi/180)*(-9) (pi/180)*(-15)  (pi/180)*26];     
% % % % % Goal_t = [(pi/180)*(-3.6313) (pi/180)*29.3335 (pi/180)*(16.9509) (pi/180)*75.8842 (pi/180)*(45.5186) (pi/180)*(-28.4714)  (pi/180)*15];
% % % % 
% % % %     d = [278 0 425 0 425 0 72];
% % % %     a = [0 0 0 0 0 0 0];
% % % %     alpha = [-90 90 -90 90 -90 90 0];
% % % %  History_Ave_Theta(:,1) = Ini_t*(180/pi);  

    
%% DH 및 초기값 정의
Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 ];
Goal_t = [(pi/180)*90 (pi/180)*0 (pi/180)*0 ];
% Goal_t = [(pi/180)*90 (pi/180)*(-50) (pi/180)*(70) ];  
    d = [0 0 0];
    a = [3 2 2];
    alpha = [0 0 0];
 History_Ave_Theta(:,1) = Ini_t*(180/pi);  
mid_point = [0.5 1];
L_end = length(mid_point);
DOF = length(Ini_t);
Coeff_att = []; Coeff_rep = [];

Local_Minimum_Timer =0; Check_Switch_Timer = 0; Escape_Timer=0; Virtual_Att_Progress_Timer=0;
Min_Point = [0;0;0]; Divid = 0;
Virtual_Switch = 0; Virtual_Num_Obs = 1; Virtual_Coef_F_rep = 10; Cre_Plot =1; Parameter_Switch = 0; Virtual_Rep_Switch=0;
Virtual_Sum_Torque_att = zeros(DOF,1); Virtual_Sum_Torque_rep = zeros(DOF,1); Virtual_Bound_con = 1;
Pre_Pre_Theta=Ini_t;   Pre_Theta = Ini_t;

for i = 1 : DOF
     for j = 1 : length(mid_point)
           Virtual_F_att{i,j}= [0;0;0] ;       %에러방지 초기값 설정
           Virtual_F_rep{i,j}= [0;0;0] ;
     end
end 

for i = 1: 10
    Ave_10_Theta(i,:) = Ini_t*0 ;
end

% 가중치
% for i = 1 : (DOF*L_end)
%     if i == 1 || i == (DOF*L_end)
%       
%         Value = 1;
% %         Value = 0;
%     else
%         
%         Value = 1;
% %         Value = 0;
%     end
%     Coeff_att = [Coeff_att Value];
% end
% 
% for i = 1 : (DOF*L_end)
%     if i == 1 || i == (DOF*L_end)
%         Value = 1;
% %         Value = 0;
%     else
%         Value = 1;
% %         Value = 0;
%     end
%     Coeff_rep = [Coeff_rep Value];
% end
Coeff_att = 0.5*[1 3 1 1 1 20];
Coeff_rep = [1 3 1 3 1 6];

Eraser = 1;
Theta = Ini_t;
Before_Check = zeros(DOF,L_end);
% 2D일때 장애물
% Obstacle{1} = [3 3 0 ; 3 4 0; 4 4 0; 4 3 0; 3 3 0];       % 1번 장애물
% 
Obs_Circle{1} = [3 4 0];
Obs_Circle{2} = [-1 5 0];
Obs_Cir_Radius = [1 0.8];

% 7축일때 장애물
% Obs_Circle{1} = [400 0 800];
% Obs_Circle{2} = [6.5 2 0];
% Obs_Cir_Radius = [100];

obs_thrs = [5 5 5 5]  ;                         % 장애물 경계값
% Transe_Obs= Obstacle{1}' ;                       % 세로로 좌표 표현
% Bound_con = Obs_Cir_Radius*1    ;                   % 경계값을 반지름으로 잡음
Bound_con = [0.3 0.1]  
Bound_con_2 = Obs_Cir_Radius*1 
 
Try_Num = 500;
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
% view(45,20)

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
% view(45,20)  % 3D 이미지 보기 좋은 각도
%%  계산
for  Repeat = 1 : Try_Num
    % 계산 전 초기값 입력

    Sum_Torque_att = zeros(DOF,1);
    Sum_Torque_att_Reverse = zeros(DOF,1);
    Sum_Torque_rep = zeros(DOF,1);
    Sum_Torque_rep_2 = zeros(DOF,1);
   Sum_Zero_F_repTorque_rep_2 = zeros(DOF,1);
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
    
    [ Switch_Obs ] = So_Judge_Obstacle( Goal_All, P_All )
    
    if 
    

    %==================================================================================================================================================================================================================
   F_Sum =[0;0;0] ;
  Theta = ((Goal_t-Ini_t)/Try_Num)*Repeat;

    %% 그래픽 부분
  So_Temp_Plot(X,Y,Z, Goal_X,Goal_Y,Goal_Z, F_Sum, Repeat, Eraser ) % 이건 원래의 그래프 장애물이 있으면 척력으로 인해 흔들림
 
%   So_Move_Plot(X,Y,Z, Goal_X,Goal_Y,Goal_Z, Ave_X,Ave_Y,Ave_Z , F_Sum, Repeat, Eraser )   % 척력에 의한 흔드림을 평균값으로 보상해줌
end






