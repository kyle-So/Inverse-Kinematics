
% 역자코비안을 이용해서 세타값을 구하는 함수
clc
clear
tic()
disp('역기구학 자코비안 해석')

% 세타가 변수일때, 그리고 t123456 은 초기값으로 입력한다.
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
t1 = 90*(pi/180)   ; d1=278        ; a1=0         ;  alpha1 =  -90;
t2 = 90*(pi/180)   ; d2=0          ; a2=0         ;  alpha2 =   90;
t3 = 90*(pi/180)   ; d3=425        ; a3=0         ;  alpha3 =  -90;
t4 = 90*(pi/180)   ; d4=0          ; a4=0         ;  alpha4 =   90;
t5 = 90*(pi/180)   ; d5=425        ; a5=0         ;  alpha5 =  -90;
t6 = 90*(pi/180)   ; d6=0          ; a6=0         ;  alpha6 =   90;
t7 = 90*(pi/180)   ; d7=72        ; a7=0          ;  alpha7 =    0;
%% NT(?) DH 임시
% t1 = 0*(pi/180)   ; d1=278        ; a1=0         ;  alpha1 =  -90;
% t2 = 0*(pi/180)   ; d2=0          ; a2=0         ;  alpha2 =   90;
% t3 = 0*(pi/180)   ; d3=425        ; a3=0         ;  alpha3 =  -90;
% t4 = 50*(pi/180)   ; d4=0          ; a4=0         ;  alpha4 =   90;
% t5 = 0*(pi/180)   ; d5=425        ; a5=0         ;  alpha5 =  90;
% t6 = -120*(pi/180)   ; d6=0          ; a6=0         ;  alpha6 =   -90;
% t7 = 20*(pi/180)   ; d7=72        ; a7=0          ;  alpha7 =    0;


% A1~A7 곱 
Sum_T=So_Show_Matrix_from_DH( t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 )

% 자코비안 값
J1=So_Show_Jacobian_form_DH(t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );

Plus_Theta = [0;0;0;0;0;0;0];
Sum_N_dP = [0;0;0;0;0;0];
Sum_N_Theta= [0;0;0;0;0;0;0];
dTheta_Normal_Sum = [0;0;0;0;0;0;0];
dP_Normal_Sum = [0;0;0;0;0;0];
Null_dTheta_Normal_Sum = [0;0;0;0;0;0;0];
Null_dP_Normal_Sum = [0;0;0;0;0;0];

% 초기로할지 결과로할지 미정. 일단 이건 위 DH 세타에 따른 포지션 -> 일단 초기값으로
Position_X =  Sum_T(1,4);
Position_Y =  Sum_T(2,4);
Position_Z =  Sum_T(3,4);

%% 절대좌표로 읽은 값.
World_RPY = So_World_ZYX( t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
disp('////////////////////////////////////////////////////////////////////////////////////')
disp('초기 X,Y,Z, x축회전, y축회전, z축회전   (단, 회전은 z축, y축, x축 순으로 회전)')
Position_Theta_Disp = [Position_X; Position_Y ; Position_Z ; World_RPY ]
Position_Theta = [Position_X; Position_Y ; Position_Z ; World_RPY*(pi/180) ];
disp('////////////////////////////////////////////////////////////////////////////////////')
disp('   ')
disp('   ')

%% 역자코비안을 위한 dP 계산
% dP = [dx; dy; dz; N;O;A]
section = 1000;
%%%%%%%%%%%%%%%%%%%% 여기만 입력해 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
Object_PTheta = [338.0716 ; 0 ; 950.2786 ; (pi/180)*(0) ; (pi/180)*0 ; (pi/180)*(0)]; % 목표값 입력   NOA는 Rad으로 적어야함.

%%%%%%%%%%%%%%%%%%%%여기까지만%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dP = (Object_PTheta - Position_Theta)/section  %결과위치 - 초기위치 를 section만큼 나눔
Theta([1 2 3 4 5 6 7],1) = [t1;t2;t3;t4;t5;t6;t7];
Null_Theta([1 2 3 4 5 6 7],1) = [t1;t2;t3;t4;t5;t6;t7];
%% 역자코비안을 이용해 Theta 계산
Anti_CN = 0;

for i=1:1:section
% for i=1:1:1
             Inv_Jaco = So_Inv_Jacobian_for_Cal(Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i),Theta(7,i) ,d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7);
             J=So_Show_Jacobian_form_DH(Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i),Theta(7,i), d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );
             Theta([1 2 3 4 5 6 7],i+1) = Theta([1 2 3 4 5 6 7],i)+ (Inv_Jaco*dP);
             % 확인차 사용한 식들.
             Order_Jaco([1,2,3,4,5,6],[1,2,3,4,5,6,7],i) =J            ; %자코비안을 순서만든거
             Order_Inv_Jaco([1,2,3,4,5,6,7],[1,2,3,4,5,6],i) = Inv_Jaco; %역자코비안 순서만든거
             
             % 그냥 일만 Pseudo 방법으로 계산한 값
             dTheta_Normal([1,2,3,4,5,6,7],i) = Inv_Jaco*dP;
             dP_Normal([1,2,3,4,5,6],i) = J * dTheta_Normal([1,2,3,4,5,6,7],i);
             dTheta_Normal_Sum = dTheta_Normal_Sum + dTheta_Normal([1,2,3,4,5,6,7],i);  %n 등분 했을 때의 세타의 증가량의 합
             dP_Normal_Sum = dP_Normal_Sum + dP_Normal([1,2,3,4,5,6],i);                %n 등분 했을 때의 이동량 P의 합
             
             
             % Null-Space 식 사용
%              disp('-----------------Null-Space 사용-------------')
             Null_Inv_Jaco = So_Inv_Jacobian_for_Cal(Null_Theta(1,i),Null_Theta(2,i),Null_Theta(3,i),Null_Theta(4,i),Null_Theta(5,i),Null_Theta(6,i),Null_Theta(7,i) ,d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7);
             Null_J=So_Show_Jacobian_form_DH(Null_Theta(1,i),Null_Theta(2,i),Null_Theta(3,i),Null_Theta(4,i),Null_Theta(5,i),Null_Theta(6,i),Null_Theta(7,i) , d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );
             Null_Theta([1 2 3 4 5 6 7],i+1) = Null_Theta([1 2 3 4 5 6 7],i)+ (Null_Inv_Jaco*dP)++(eye(7)-Null_Inv_Jaco*Null_J)*[1;1;1;1;1;1;1]*0.01;
             Null_dTheta_Normal([1,2,3,4,5,6,7],i) = Null_Inv_Jaco*dP+((eye(7)-Null_Inv_Jaco*Null_J)*[1;1;1;1;1;1;1]*10)+10;
             Null_dP_Normal([1,2,3,4,5,6],i) = Null_J * Null_dTheta_Normal([1,2,3,4,5,6,7],i);
             Null_dTheta_Normal_Sum = Null_dTheta_Normal_Sum + Null_dTheta_Normal([1,2,3,4,5,6,7],i);        
             Null_dP_Normal_Sum = Null_dP_Normal_Sum + Null_dP_Normal([1,2,3,4,5,6],i);       
              
             Check_Null = Null_J*Null_Inv_Jaco;
end


%% 검산값

% for 문 값 출력
disp('◇◇◇◇◇◇◇◇◇◇◇◇◇역자코 * dP 사용◇◇◇◇◇◇◇◇◇◇◇◇◇')
dTheta_Normal;
dP_Normal;
dTheta_Normal_Sum;
dP_Normal_Sum;
disp('기존각도 + 역자코 * dP')
Angle =Theta(:,section+1) * 180/ pi
disp('기존각도 + 역자코 * dP 를 이용해 구한 세타갑을 Forward 한 결과')
for i = 1 : 1 : section +1
    [Re_SUM_T,World_Rotate] = So_Position_and_Theta_Cal( Theta(1,i), Theta(2,i), Theta(3,i), Theta(4,i), Theta(5,i), Theta(6,i),Theta(7,i), d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,alpha7 );

end
Normal_Position = Re_SUM_T([1,2,3],4)
disp('◇◇◇◇◇◇◇◇◇◇◇◇◇역자코 * dP 사용 여기 까지◇◇◇◇◇◇◇◇◇◇◇◇◇')

disp(' ')
disp(' ')
disp('★★★★★★★★★★★★★Null-Space 이용 값★★★★★★★★★★★★★★★★')
Null_dTheta_Normal;
Null_dP_Normal;
Null_dTheta_Normal_Sum;
Null_dP_Normal_Sum;
Angle_2Null= Null_Theta(:,section+1) * 180/ pi
disp('기존각도 + 역자코 * dP 를 이용해 구한 세타갑을 Forward 한 결과')
for i = 1 : 1 : section +1
    [Re_SUM_T2,World_Rotate] = So_Position_and_Theta_Cal( Null_Theta(1,i),Null_Theta(2,i),Null_Theta(3,i),Null_Theta(4,i),Null_Theta(5,i),Null_Theta(6,i),Null_Theta(7,i), d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,alpha7 );
end
Result_Angle2_Null=Re_SUM_T2([1,2,3],4)
disp('★★★★★★★★★★★★★Null-Space 이용 값 끝★★★★★★★★★★★★★★★★')

%% 얻은 세타를 라디안 디그리로 표현.

Rad_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1) ; Theta(7,section+1)];
disp('   ')
disp('=====================================================================')
disp('역자코비안을 이용해 구한 세타1~세타6')
Deg_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1);Theta(7,section+1)] * (180/pi)
disp('추가 각도')
disp('=====================================================================')
disp('   ')
















%% 여기부터 검산 과정
[Re_SUM_T,World_Rotate] = So_Position_and_Theta_Cal( Theta(1,section+1), Theta(2,section+1), Theta(3,section+1), Theta(4,section+1), Theta(5,section+1), Theta(6,section+1),Theta(7,section+1), d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,alpha7 );
Re_SUM_T
% [Other_Re_SUM_T,Other_World_Rotate] = So_Position_and_Theta_Cal( New_Theta(1), New_Theta(2),  New_Theta(3),  New_Theta(4),  New_Theta(5),  New_Theta(6),  New_Theta(7), d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,alpha7 );
% Other_Re_T = Other_Re_SUM_T
Position_R_X = Re_SUM_T(1,4);
Position_R_Y = Re_SUM_T(2,4);
Position_R_Z = Re_SUM_T(3,4);
disp('   ')
disp('   ')
disp('*********************************************************************************')
disp('위에서 구한 세타1~세타6을 Forward Kinematics한 결과')
Position_Theta = [Position_R_X; Position_R_Y ; Position_R_Z ; World_Rotate ]
disp('*********************************************************************************')


%% 그래프 그리기
%   세타 모양 나오기


% % % % % % %      for i = 1:section+1
% % % % % % %       
% % % % % % %               t1 = Theta(1,i);
% % % % % % %               t2 = Theta(2,i);
% % % % % % %               t3 = Theta(3,i);
% % % % % % %               t4 = Theta(4,i);
% % % % % % %               t5 = Theta(5,i);
% % % % % % %               t6 = Theta(6,i);
% % % % % % %               t7 = Theta(7,i);
% % % % % % %                 [Plot_A1, Plot_A2, Plot_A3, Plot_A4, Plot_A5, Plot_A6, Plot_A7] = So_Matrix_for_Cal(  t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );
% % % % % % %               
% % % % % % %                       A1 = Plot_A1    ;
% % % % % % %                       A12 = Plot_A2;
% % % % % % %                       A123 = Plot_A3;
% % % % % % %                       A1234 = Plot_A4;
% % % % % % %                       A12345 = Plot_A5;
% % % % % % %                       A123456 = Plot_A6;
% % % % % % %                       A1234567 = Plot_A7;
% % % % % % %        
% % % % % % % 
% % % % % % %         Plot_X(i,2) = roundn(A1(1,4),-3); % 1축 X축 위치
% % % % % % %         Plot_Y(i,2) = roundn(A1(2,4),-3); % 1축 Y축 위치
% % % % % % %         Plot_Z(i,2) = roundn(A1(3,4),-3); % 1축 Z축 위치
% % % % % % %         Plot_X(i,3) = roundn(A12(1,4),-3); % 2축 X축 위치
% % % % % % %         Plot_Y(i,3) = roundn(A12(2,4),-3); % 2축 Y축 위치
% % % % % % %         Plot_Z(i,3) = roundn(A12(3,4),-3); % 2축 Z축 위치
% % % % % % %         Plot_X(i,4) = roundn(A123(1,4),-3); % 3축 X축 위치
% % % % % % %         Plot_Y(i,4) = roundn(A123(2,4),-3); % 3축 Y축 위치
% % % % % % %         Plot_Z(i,4) = roundn(A123(3,4),-3); % 3축 Z축 위치   
% % % % % % %         Plot_X(i,5) = roundn(A1234(1,4),-3); % 4축 X축 위치
% % % % % % %         Plot_Y(i,5) = roundn(A1234(2,4),-3); % 4축 Y축 위치
% % % % % % %         Plot_Z(i,5) = roundn(A1234(3,4),-3); % 4축 Z축 위치 
% % % % % % %         Plot_X(i,6) = roundn(A12345(1,4),-3); % 5축 X축 위치
% % % % % % %         Plot_Y(i,6) = roundn(A12345(2,4),-3); % 5축 Y축 위치
% % % % % % %         Plot_Z(i,6) = roundn(A12345(3,4),-3); % 5축 Z축 위치    
% % % % % % %         Plot_X(i,7) = roundn(A123456(1,4),-3) ;% 6축 X축 위치
% % % % % % %         Plot_Y(i,7) = roundn(A123456(2,4),-3); % 6축 Y축 위치
% % % % % % %         Plot_Z(i,7) = roundn(A123456(3,4),-3); % 6축 Z축 위치   
% % % % % % %         Plot_X(i,8) = roundn(A1234567(1,4),-3); % 7축 X축 위치
% % % % % % %         Plot_Y(i,8) = roundn(A1234567(2,4),-3); % 7축 Y축 위치
% % % % % % %         Plot_Z(i,8) = roundn(A1234567(3,4),-3); % 7축 Z축 위치    
% % % % % % %         
% % % % % % %      end
% % % % % % % 
% % % % % % %         hold on
% % % % % % %         Plot_Count = 0;
% % % % % % %        for i=1: (section)/10 :section
% % % % % % %             
% % % % % % %                  if Plot_Count <= 1
% % % % % % %                        plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'k-')
% % % % % % %                        plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'ko-')
% % % % % % %                        Plot_Count = Plot_Count +1;
% % % % % % %                  elseif Plot_Count <= 3
% % % % % % %                        plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'m-')
% % % % % % %                        plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'mo-')
% % % % % % %                        Plot_Count = Plot_Count +1;
% % % % % % %                  elseif Plot_Count <= 5
% % % % % % %                     plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'g-')
% % % % % % %                     plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'go-')
% % % % % % %                     Plot_Count = Plot_Count +1;
% % % % % % %                  elseif Plot_Count <= 7
% % % % % % %                     plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'c-')
% % % % % % %                     plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'co-')
% % % % % % %                     Plot_Count = Plot_Count +1  ;         
% % % % % % %                  else 
% % % % % % %                     plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'b-') 
% % % % % % %                     plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'bo-')
% % % % % % %                     Plot_Count = Plot_Count +1 ;
% % % % % % %                  end
% % % % % % %                  xlabel('X 축')
% % % % % % %                  ylabel('Y 축')
% % % % % % %                  zlabel('Z 축')
% % % % % % %                  grid
% % % % % % %         axis([-500 500 -500 500 -500 500])
% % % % % % %        end
% % % % % % %        
% % % % % % %         plot3(Plot_X(section+1,:),Plot_Y(section+1,:), Plot_Z(i,:) , 'r-')
% % % % % % %         plot3(Plot_X(section+1,8),Plot_Y(section+1,8), Plot_Z(i,8) , 'r-*')
% % % % % % %        toc()
% %% 책에 나와있는 대수학적 해
% % 위에서 얻은 A1~A6까지의 행렬에 최종값 세타를 넣은 A123456 변수를 소환한다.
% % 
% % IT = A123456
% % 
% % Th1 = atan( IT(2,4)/IT(1,4)  )  ;
% % 
% % Th234 = atan( IT(3,3) / (cos(Th1)*IT(1,3)+sin(Th1)*IT(2,3))  );
% % 
% % C3 = ( ( (IT(1,4)*cos(Th1) +IT(2,4)*sin(Th1) -cos(Th234)*a4  )^2 + ( IT(3,4) - sin(Th234)*a4     )^2 - a2^2 -a3^2     )        /  2*a2*a3           ) ;
% % S3 = abs(sqrt(1-C3^2)) ;
% % Th3 = atan ( S3 / C3  );
% % 
% % Th2 = atan(  (  (C3*a3+a2) *  (IT(3,4)-sin(Th234)*a4) - sin(Th3)*a3*(IT(1,4)*cos(Th1)+IT(2,4)*sin(Th1)-cos(Th234)*a4)         )         /         ( (C3*a3+a2) * ( IT(1,4)*cos(Th1) + IT(2,4)*sin(Th1) - cos(Th234)*a4  ) + S3*a3*(IT(3,4)-sin(Th234)*a4)                 )            );
% % 
% % Th4 = Th234 - Th2 - Th3  ;
% % 
% % 
% % Th5 = atan(  (   cos(Th234) * ( cos(Th1)*IT(1,3)+sin(Th1)*IT(2,3) + sin(Th1)*IT(2,3) ) + sin(Th234)*IT(3,3)    )     /     ( sin(Th1)*IT(1,3) -  cos(Th1)*IT(2,3) )     );
% % 
% % Th6 = atan(  (-sin(Th234) * ( cos(Th1)*IT(1,1) + sin(Th1)*IT(2,1)  ) + cos(Th234)*IT(3,1)   )   /  ( -sin(Th234) * (cos(Th1)*IT(1,2)+sin(Th1)*IT(2,2)) + cos(Th234)*IT(3,2)   )   ) ;
% % 
% % 
% % All_Theta = [Th1; Th2 ;Th3 ;Th4 ;Th5 ;Th6 ]
% % All_Theta_Deg = [Th1; Th2 ;Th3 ;Th4 ;Th5 ;Th6 ] * (180/pi)
% 
% %% 검산
% % [All,RX,RY,RZ]=So_Position_and_Theta_Cal( Th1 ,Th2 ,Th3 ,Th4 ,Th5 ,Th6 , d1,d2,d3,d4,d5,d6, a1,a2,a3,a4,a5,a6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 )
% 


