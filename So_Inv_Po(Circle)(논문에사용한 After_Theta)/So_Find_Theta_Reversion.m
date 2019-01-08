
% 역자코비안을 이용해서 세타값을 구하는 함수
clc
clear

disp('역기구학 자코비안 해석')

% 세타가 변수일때, 그리고 t123456 은 초기값으로 입력한다.
% DH 초기값 겸, 세타 초기값
%******************************************************
t1 = pi/2         ; d1=0          ; a1=0         ;  alpha1 =  90;
t2 = pi/2*0         ; d2=0          ; a2=1         ;  alpha2 =   0;
t3 = pi/2         ; d3=0          ; a3=1         ;  alpha3 =   0;
t4 = pi/4*0         ; d4=0          ; a4=1         ;  alpha4 = -90;
t5 = pi/2         ; d5=0          ; a5=0         ;  alpha5 =  90;
t6 = -(pi*3)/4*0     ; d6=0          ; a6=0         ;  alpha6 =   0;
%******************************************************

A1 =  [cos(t1) -sin(t1)*cosd(alpha1) sin(t1)*sind(alpha1) a1*cos(t1);
       sin(t1) cos(t1)*cosd(alpha1)  -cos(t1)*sind(alpha1) a1*sin(t1);
       0            sind(alpha1)            cosd(alpha1)         d1;
       0                0                 0                1];
   
A2 =  [cos(t2) -sin(t2)*cosd(alpha2) sin(t2)*sind(alpha2) a2*cos(t2);
       sin(t2) cos(t2)*cosd(alpha2)  -cos(t2)*sind(alpha2) a2*sin(t2);
       0            sind(alpha2)            cosd(alpha2)         d2;
       0                0                 0                1];
A3 =  [cos(t3) -sin(t3)*cosd(alpha3) sin(t3)*sind(alpha3) a3*cos(t3);
       sin(t3) cos(t3)*cosd(alpha3)  -cos(t3)*sind(alpha3) a3*sin(t3);
       0            sind(alpha3)            cosd(alpha3)         d3;
       0                0                 0                1];
   
A4 =  [cos(t4) -sin(t4)*cosd(alpha4) sin(t4)*sind(alpha4) a4*cos(t4);
       sin(t4) cos(t4)*cosd(alpha4)  -cos(t4)*sind(alpha4) a4*sin(t4);
       0            sind(alpha4)            cosd(alpha4)         d4;
       0                0                 0                1] ; 
   
A5 =  [cos(t5) -sin(t5)*cosd(alpha5) sin(t5)*sind(alpha5) a5*cos(t5);
       sin(t5) cos(t5)*cosd(alpha5)  -cos(t5)*sind(alpha5) a5*sin(t5);
       0            sind(alpha5)            cosd(alpha5)         d5;
       0                0                 0                1];   

A6 =  [cos(t6) -sin(t6)*cosd(alpha6) sin(t6)*sind(alpha6) a6*cos(t6);
       sin(t6) cos(t6)*cosd(alpha6)  -cos(t6)*sind(alpha6) a6*sin(t6);
       0            sind(alpha6)            cosd(alpha6)         d6;
       0                0                 0                1];
   
Sum_T=(A1*A2*A3*A4*A5*A6)

% 초기로할지 결과로할지 미정. 일단 이건 위 DH 세타에 따른 포지션 -> 일단 초기값으로
Position_X =  cos(t1)*(a3*cos(t2 + t3) + a2*cos(t2) + a4*cos(t2 + t3 + t4));
Position_Y =  sin(t1)*(a3*cos(t2 + t3) + a2*cos(t2) + a4*cos(t2 + t3 + t4));
Position_Z =  a3*sin(t2 + t3) + a2*sin(t2) + a4*sin(t2 + t3 + t4);

% 초기 각도 XYZ축에 대한 NOA
NOA([1 2 3],[1 2 3]) = Sum_T([1 2 3],[1 2 3]);

Ini_Delta_Z_A = atan2(  NOA(2,1) , NOA(1,1) );
Ini_Delta_Y_O = atan2( -NOA(3,1) , NOA(1,1)*cos(Ini_Delta_Z_A)+NOA(2,1)*sin(Ini_Delta_Z_A) );
Ini_Delta_X_N = atan2( -NOA(2,3)*cos(Ini_Delta_Z_A)+NOA(1,3)*sin(Ini_Delta_Z_A), NOA(2,2)*cos(Ini_Delta_Z_A)-NOA(1,2)*sin(Ini_Delta_Z_A));

Rotate_X = So_Rad_to_Deg( Ini_Delta_X_N);
Rotate_Y = So_Rad_to_Deg( Ini_Delta_Y_O);
Rotate_Z = So_Rad_to_Deg( Ini_Delta_Z_A);


disp('////////////////////////////////////////////////////////////////////////////////////')
disp('초기 X,Y,Z, x축회전, y축회전, z축회전   (단, 회전은 z축, y축, x축 순으로 회전)')
Position_Theta = [Position_X; Position_Y ; Position_Z ; Ini_Delta_X_N ; Ini_Delta_Y_O ;Ini_Delta_Z_A ]
disp('////////////////////////////////////////////////////////////////////////////////////')
disp('   ')
disp('   ')

%% 역자코비안을 위한 dP 계산
% dP = [dx; dy; dz; N;O;A]
section = 100;
%%%%%%%%%%%%%%%%%%%% 여기만 입력해 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
Object_PTheta = [1 ; 1 ; 1 ; pi/4*0 ; pi/4*0 ; pi/4]; % 목표값 입력   NOA는 Rad으로 적어야함.

%%%%%%%%%%%%%%%%%%%%여기까지만%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dP = (Object_PTheta - Position_Theta)/section   %결과위치 - 초기위치 를 section만큼 나눔
Theta([1 2 3 4 5 6],1) = [t1;t2;t3;t4;t5;t6];

%% 역자코비안을 이용해 Theta 계산

for i=1:section
             Inv_Jaco = So_Inv_Jacobian_for_Cal(Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i),a2,a3,a4);
             Theta([1 2 3 4 5 6],i+1) = Theta([1 2 3 4 5 6],i)+ (Inv_Jaco*dP);
end



%얻은 세타를 라디안 디그리로 표현.

Rad_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1)]
disp('   ')
disp('=====================================================================')
disp('역자코비안을 이용해 구한 세타1~세타6')
Deg_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1)] * (180/pi)
disp('=====================================================================')
disp('   ')

%% 여기부터 검산 과정
[Re_SUM_T,Ini_Ro_X, Ini_Ro_Y,Ini_Ro_Z] = So_Position_and_Theta_Cal( Theta(1,section+1), Theta(2,section+1), Theta(3,section+1), Theta(4,section+1), Theta(5,section+1), Theta(6,section+1), d1,d2,d3,d4,d5,d6, a1,a2,a3,a4,a5,a6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 );
Position_R_X = Re_SUM_T(1,4);
Position_R_Y = Re_SUM_T(2,4);
Position_R_Z = Re_SUM_T(3,4);
disp('   ')
disp('   ')
disp('*********************************************************************************')
disp('위에서 구한 세타1~세타6을 Forward Kinematics한 결과')
Position_Theta = [Position_R_X; Position_R_Y ; Position_R_Z ; Ini_Ro_X ; Ini_Ro_Y ;Ini_Ro_Z ]
disp('*********************************************************************************')

%% 그래프 그리기
  %세타 모양 나오기

  syms t1 t2 t3 t4 t5 t6

  

         
     for i = 1:section+1
      
              t1 = Theta(1,i);
              t2 = Theta(2,i);
              t3 = Theta(3,i);
              t4 = Theta(4,i);
              t5 = Theta(5,i);
              t6 = Theta(6,i);
              
                [Plot_A1, Plot_A2, Plot_A3, Plot_A4, Plot_A5, Plot_A6] = So_Matrix_for_Cal(  Theta(1,i), Theta(2,i), Theta(3,i), Theta(4,i), Theta(5,i), Theta(6,i)  );
              
                      A1 = Plot_A1    ;
                      A12 = Plot_A2;
                      A123 = Plot_A3;
                      A1234 = Plot_A4;
                      A12345 = Plot_A5;
                      A123456 = Plot_A6;
                      
        Plot_X(i,2) = roundn(A1(1,4),-3); % 1축 X축 위치
        Plot_Y(i,2) = roundn(A1(2,4),-3); % 1축 Y축 위치
        Plot_Z(i,2) = roundn(A1(3,4),-3); % 1축 Z축 위치
        Plot_X(i,3) = roundn(A12(1,4),-3); % 2축 X축 위치
        Plot_Y(i,3) = roundn(A12(2,4),-3); % 2축 Y축 위치
        Plot_Z(i,3) = roundn(A12(3,4),-3); % 2축 Z축 위치
        Plot_X(i,4) = roundn(A123(1,4),-3); % 3축 X축 위치
        Plot_Y(i,4) = roundn(A123(2,4),-3); % 3축 Y축 위치
        Plot_Z(i,4) = roundn(A123(3,4),-3); % 3축 Z축 위치   
        Plot_X(i,5) = roundn(A1234(1,4),-3); % 4축 X축 위치
        Plot_Y(i,5) = roundn(A1234(2,4),-3); % 4축 Y축 위치
        Plot_Z(i,5) = roundn(A1234(3,4),-3); % 4축 Z축 위치 
        Plot_X(i,6) = roundn(A12345(1,4),-3); % 5축 X축 위치
        Plot_Y(i,6) = roundn(A12345(2,4),-3); % 5축 Y축 위치
        Plot_Z(i,6) = roundn(A12345(3,4),-3); % 5축 Z축 위치    
        Plot_X(i,7) = roundn(A123456(1,4),-3) ;% 6축 X축 위치
        Plot_Y(i,7) = roundn(A123456(2,4),-3); % 6축 Y축 위치
        Plot_Z(i,7) = roundn(A123456(3,4),-3); % 6축 Z축 위치   
  
     end
       
        hold on
        Plot_Count = 0;
       for i=1: (section)/10 :section
            
                 if Plot_Count <= 1
                       plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'ko-')
                       Plot_Count = Plot_Count +1;
                 elseif Plot_Count <= 3
                       plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'m*-') 
                       Plot_Count = Plot_Count +1;
                 elseif Plot_Count <= 5
                    plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'gp-')  
                    Plot_Count = Plot_Count +1;
                 elseif Plot_Count <= 7
                    plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'cp-')  
                    Plot_Count = Plot_Count +1  ;         
                 else 
                    plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'bd-')  
                    Plot_Count = Plot_Count +1 ;
                 end
                 xlabel('X 축')
                 ylabel('Y 축')
                 zlabel('Z 축')
                 grid
        axis([-0.5 2 -0.5 2 -1 2])
       end
       
        plot3(Plot_X(section+1,:),Plot_Y(section+1,:), Plot_Z(i,:) , 'rv-')
       
% %% 책에 나와있는 대수학적 해
% % 위에서 얻은 A1~A6까지의 행렬에 최종값 세타를 넣은 A123456 변수를 소환한다.
% 
% IT = A123456
% 
% Th1 = atan( IT(2,4)/IT(1,4)  )  ;
% 
% Th234 = atan( IT(3,3) / (cos(Th1)*IT(1,3)+sin(Th1)*IT(2,3))  );
% 
% C3 = ( ( (IT(1,4)*cos(Th1) +IT(2,4)*sin(Th1) -cos(Th234)*a4  )^2 + ( IT(3,4) - sin(Th234)*a4     )^2 - a2^2 -a3^2     )        /  2*a2*a3           ) ;
% S3 = abs(sqrt(1-C3^2)) ;
% Th3 = atan ( S3 / C3  );
% 
% Th2 = atan(  (  (C3*a3+a2) *  (IT(3,4)-sin(Th234)*a4) - sin(Th3)*a3*(IT(1,4)*cos(Th1)+IT(2,4)*sin(Th1)-cos(Th234)*a4)         )         /         ( (C3*a3+a2) * ( IT(1,4)*cos(Th1) + IT(2,4)*sin(Th1) - cos(Th234)*a4  ) + S3*a3*(IT(3,4)-sin(Th234)*a4)                 )            );
% 
% Th4 = Th234 - Th2 - Th3  ;
% 
% 
% Th5 = atan(  (   cos(Th234) * ( cos(Th1)*IT(1,3)+sin(Th1)*IT(2,3) + sin(Th1)*IT(2,3) ) + sin(Th234)*IT(3,3)    )     /     ( sin(Th1)*IT(1,3) -  cos(Th1)*IT(2,3) )     );
% 
% Th6 = atan(  (-sin(Th234) * ( cos(Th1)*IT(1,1) + sin(Th1)*IT(2,1)  ) + cos(Th234)*IT(3,1)   )   /  ( -sin(Th234) * (cos(Th1)*IT(1,2)+sin(Th1)*IT(2,2)) + cos(Th234)*IT(3,2)   )   ) ;
% 
% 
% All_Theta = [Th1; Th2 ;Th3 ;Th4 ;Th5 ;Th6 ]
% All_Theta_Deg = [Th1; Th2 ;Th3 ;Th4 ;Th5 ;Th6 ] * (180/pi)
% 
% %% 검산
% [All,RX,RY,RZ]=So_Position_and_Theta_Cal( Th1 ,Th2 ,Th3 ,Th4 ,Th5 ,Th6 , d1,d2,d3,d4,d5,d6, a1,a2,a3,a4,a5,a6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 )
% 


