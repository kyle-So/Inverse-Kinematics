function [ Return_Theta1, Return_Theta2, Return_Theta3, Return_Theta4, Return_Theta5, Return_Theta6, Return_Theta7 ] = So_Rotate_X(  X,Y,Z, Yaw, ThetaY1,ThetaY2,ThetaY3,ThetaY4,ThetaY5,ThetaY6,ThetaY7 )
%SO_ROTATE_X Summary of this function goes here
%   Detailed explanation goes here
% DH 초기값 겸, 세타 초기값
%******************************************************
% t1    ; d1=1          ; a1=0         ;  alpha1 =   90;
% t2    ; d2=0          ; a2=0         ;  alpha2 =  -90;
% t3    ; d3=3          ; a3=0         ;  alpha3 =   90;
% t4    ; d4=0          ; a4=0         ;  alpha4 =  -90;
% t5    ; d5=3          ; a5=0         ;  alpha5 =   90;
% t6    ; d6=0          ; a6=0         ;  alpha6 =  -90;
% t7    ; d7=1          ; a7=0         ;  alpha7 =    0;
%******************************************************
% 
%% NT(?) DH 임시
t1 = ThetaY1   ; d1=278          ; a1=0         ;  alpha1 =  -90;
t2 = ThetaY2   ; d2=0            ; a2=0         ;  alpha2 =   90;
t3 = ThetaY3   ; d3=425          ; a3=0         ;  alpha3 =  -90;
t4 = ThetaY4   ; d4=0            ; a4=0         ;  alpha4 =   90;
t5 = ThetaY5   ; d5=425          ; a5=0         ;  alpha5 =  -90;
t6 = ThetaY6   ; d6=0            ; a6=0         ;  alpha6 =   90;
t7 = ThetaY7   ; d7=72           ; a7=0         ;  alpha7 =    0;

%% 논문 7축 해보는거
% t1 = 0*(pi/180)   ; d1=0          ; a1=0         ;  alpha1 =  -90;
% t2 = 0*(pi/180)   ; d2=0          ; a2=1         ;  alpha2 =    0;
% t3 = 0*(pi/180)   ; d3=0          ; a3=1         ;  alpha3 =    0;
% t4 = 0*(pi/180)   ; d4=0          ; a4=0         ;  alpha4 =   90;
% t5 = 0*(pi/180)   ; d5=3          ; a5=0         ;  alpha5 =  -90;
% t6 = 0*(pi/180)   ; d6=0          ; a6=0         ;  alpha6 =   90;
% t7 = 0*(pi/180)   ; d7=3          ; a7=0         ;  alpha7 =    0;
% A1~A7 곱 
Sum_T=So_Show_Matrix_from_DH( t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 )

% 자코비안 값
So_Show_Jacobian_form_DH(t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );


% 초기로할지 결과로할지 미정. 일단 이건 위 DH 세타에 따른 포지션 -> 일단 초기값으로
Position_X =  Sum_T(1,4);
Position_Y =  Sum_T(2,4);
Position_Z =  Sum_T(3,4);

% 초기 각도 XYZ축에 대한 NOA
% NOA([1 2 3],[1 2 3]) = Sum_T([1 2 3],[1 2 3]);
% 
% Ini_Delta_Z_A = atan2(  NOA(2,1) , NOA(1,1) );
% Ini_Delta_Y_O = atan2( -NOA(3,1) , NOA(1,1)*cos(Ini_Delta_Z_A)+NOA(2,1)*sin(Ini_Delta_Z_A) );
% Ini_Delta_X_N = atan2( -NOA(2,3)*cos(Ini_Delta_Z_A)+NOA(1,3)*sin(Ini_Delta_Z_A), NOA(2,2)*cos(Ini_Delta_Z_A)-NOA(1,2)*sin(Ini_Delta_Z_A));
% 
% Rotate_X = So_Rad_to_Deg( Ini_Delta_X_N);
% Rotate_Y = So_Rad_to_Deg( Ini_Delta_Y_O);
% Rotate_Z = So_Rad_to_Deg( Ini_Delta_Z_A);
% 
% disp('////////////////////////////////////////////////////////////////////////////////////')
% disp('초기 X,Y,Z, x축회전, y축회전, z축회전   (단, 회전은 z축, y축, x축 순으로 회전)')
% Position_Theta = [Position_X; Position_Y ; Position_Z ; Ini_Delta_X_N ; Ini_Delta_Y_O ;Ini_Delta_Z_A ]
% disp('////////////////////////////////////////////////////////////////////////////////////')
% disp('   ')
% disp('   ')

%% 절대좌표로 읽은 값.

World_RPY = So_World_ZYX( t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );


disp('////////////////////////////////////////////////////////////////////////////////////')
disp('초기 X,Y,Z, x축회전, y축회전, z축회전   (단, 회전은 z축, y축, x축 순으로 회전)')
Position_Theta = [Position_X; Position_Y ; Position_Z ; World_RPY*(pi/180) ]
disp('////////////////////////////////////////////////////////////////////////////////////')
disp('   ')
disp('   ')




%% 역자코비안을 위한 dP 계산
% dP = [dx; dy; dz; N;O;A]
section = 1000;
%%%%%%%%%%%%%%%%%%%% 여기만 입력해 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
Object_PTheta = [ X ; Y ; Z ; (pi/180)*Yaw ; (pi/180)*World_RPY(2) ; World_RPY(3)*(pi/180)]; % 목표값 입력   NOA는 Rad으로 적어야함.

%%%%%%%%%%%%%%%%%%%%여기까지만%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dP = (Object_PTheta - Position_Theta)/section  %결과위치 - 초기위치 를 section만큼 나눔
Theta([1 2 3 4 5 6 7],1) = [t1;t2;t3;t4;t5;t6;t7];

%% 역자코비안을 이용해 Theta 계산
Anti_CN = 0;

for i=1:section
             Inv_Jaco = So_Inv_Jacobian_for_Cal(Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i),Theta(7,i) ,d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7);
             
            
%              if cond(Inv_Jaco) < 100
% %                  disp('====================================================')
%                 i             ;                                  %i번째
%                 Theta_Check = Theta([1 2 3 4 5 6 7],i) *(180/pi);  %각도
%                 Condition_Number = cond(Inv_Jaco)     ;          %조건수
%                 DH_ing = So_Show_Matrix_from_DH(Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i), Theta(7,i) , d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
%                 [XYZ_Under,RPY_Under] = So_Position_and_Theta_Cal( Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i), Theta(7,i) , d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
%                 XYZRPY([1 2 3 4 5 6],1) = [XYZ_Under([1 2 3],4);RPY_Under] ;
%                 Inv_Jaco_Under = Inv_Jaco ;                     %역자코비안
%                 Before_Inv_Jaco = Inv_Jaco ;                    %조건수 넘을경우의 보존된 자코비안
%                 Before_Theta = Theta([1 2 3 4 5 6 7],i);          %조건수 넘을경우의 보존된 라디안 각도
%                 Show_Before_Theta = Before_Theta *(180/pi)   ;    %보존된 각도 세타로 출력 
%                 
%                 dP_Times = 1;
                Theta([1 2 3 4 5 6 7],i+1) = Theta([1 2 3 4 5 6 7],i)+ (Inv_Jaco*dP);
%                 
% %                 disp('====================================================')
%              else
% %                  disp(' ')
% %                  disp('####################################################')
% %                  disp(' ')
% %                  disp('Condition-Number 기준값 초과')
%                  i             ;                                      %i번째
%                 Theta_Check_Over = Theta([1 2 3 4 5 6 7],i) *(180/pi) ; %각도
%                 Condition_Number_Over = cond(Inv_Jaco)    ;           %조건수
%                 Inv_Jaco_Over = Inv_Jaco;                            %역자코비안
%                 Before_Inv_Jaco_Over = Before_Inv_Jaco;              %조건수 넘을경우 보존된 자코비안
%                 Before_Theta_Over = Before_Theta;                    %조건수 넘어서 그 전 세타 라디안으로 불러옴
%                 Show_Before_Theta_Over = Before_Theta_Over *(180/pi); %라디안값을 각도로
%     
%                 dP_Times = dP_Times + 1;
%                 Theta([1 2 3 4 5 6 7],i+1) = Before_Theta+ (Before_Inv_Jaco*dP)*dP_Times;
%                                
% %                 disp('####################################################')
%                 
%                 Anti_CN = Anti_CN+1    ;         
%              end
             
end

             DH_ing = So_Show_Matrix_from_DH( Theta(1,section+1),Theta(2,section+1),Theta(3,section+1),Theta(4,section+1),Theta(5,section+1),Theta(6,section+1),Theta(7, section+1) , d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
             DH_Sum([1 2 3],section+1) = DH_ing([1 2 3],4);
             DH_Rotate([1 2 3],section+1) = So_World_ZYX( Theta(1,section+1),Theta(2,section+1),Theta(3,section+1),Theta(4,section+1),Theta(5,section+1),Theta(6,section+1),Theta(7, section+1) , d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
             Sum_PT([1 2 3 4 5 6],section+1)=[DH_Sum([1 2 3],section+1);DH_Rotate([1 2 3],section+1)];



%얻은 세타를 라디안 디그리로 표현.

Rad_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1) ; Theta(7,section+1)]
disp('   ')
disp('=====================================================================')
disp('역자코비안을 이용해 구한 세타1~세타6')
Deg_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1);Theta(7,section+1)] * (180/pi)
disp('=====================================================================')
disp('   ')

%% 반환값 정의
Return_Theta1 = Theta(1,section+1);
Return_Theta2 = Theta(2,section+1);
Return_Theta3 = Theta(3,section+1);
Return_Theta4 = Theta(4,section+1);
Return_Theta5 = Theta(5,section+1);
Return_Theta6 = Theta(6,section+1);
Return_Theta7 = Theta(7,section+1);
%% 여기부터 검산 과정
[Re_SUM_T,World_Rotate] = So_Position_and_Theta_Cal( Theta(1,section+1), Theta(2,section+1), Theta(3,section+1), Theta(4,section+1), Theta(5,section+1), Theta(6,section+1),Theta(7,section+1), d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,alpha7 );
Re_SUM_T
Position_R_X = Re_SUM_T(1,4);
Position_R_Y = Re_SUM_T(2,4);
Position_R_Z = Re_SUM_T(3,4);
disp('   ')
disp('   ')
disp('*********************************************************************************')
disp('원하는 목표값을 얻기 위한 세타값')
Get_Theta = [Theta(1,section+1);Theta(2,section+1);Theta(3,section+1);Theta(4,section+1);Theta(5,section+1);Theta(6,section+1);Theta(7,section+1) ] * (180/pi)
disp('   ')
disp('위에서 구한 세타1~세타6을 Forward Kinematics한 결과')
Position_Theta = [Position_R_X; Position_R_Y ; Position_R_Z ; World_Rotate ]
disp('*********************************************************************************')

end


