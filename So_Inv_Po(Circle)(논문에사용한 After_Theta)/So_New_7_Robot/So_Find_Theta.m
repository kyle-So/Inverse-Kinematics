
% �����ں���� �̿��ؼ� ��Ÿ���� ���ϴ� �Լ�
clc
clear
tic()
disp('���ⱸ�� ���ں�� �ؼ�')

% ��Ÿ�� �����϶�, �׸��� t123456 �� �ʱⰪ���� �Է��Ѵ�.
% DH �ʱⰪ ��, ��Ÿ �ʱⰪ
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
%% NT(?) DH �ӽ�
% t1 = 2.5552   ; d1=1          ; a1=0         ;  alpha1 =   90;
% t2 = 2.0798   ; d2=0          ; a2=0         ;  alpha2 =  -90;
% t3 = 1.2416   ; d3=3          ; a3=0         ;  alpha3 =   90;
% t4 = 2.5637   ; d4=0          ; a4=0         ;  alpha4 =  -90;
% t5 = 1.0241   ; d5=3          ; a5=0         ;  alpha5 =   90;
% t6 = 1.3137   ; d6=0          ; a6=0         ;  alpha6 =  -90;
% t7 = -0.4213   ; d7=1          ; a7=0         ;  alpha7 =    0;

t1 = 90*(pi/180)   ; d1=278        ; a1=0         ;  alpha1 =   90;
t2 = 90*(pi/180)   ; d2=0          ; a2=0         ;  alpha2 =  -90;
t3 = 90*(pi/180)   ; d3=425        ; a3=0         ;  alpha3 =   90;
t4 = 90*(pi/180)   ; d4=0          ; a4=0         ;  alpha4 =  -90;
t5 = 90*(pi/180)   ; d5=326        ; a5=0         ;  alpha5 =   90;
t6 = 90*(pi/180)   ; d6=0          ; a6=0         ;  alpha6 =  -90;
t7 = 90*(pi/180)   ; d7=171        ; a7=0         ;  alpha7 =    0;

%% �� 7�� �غ��°�
% t1 = 90*(pi/180)   ; d1=0          ; a1=0         ;  alpha1 =  -90;
% t2 = 0*(pi/180)   ; d2=0          ; a2=1         ;  alpha2 =    0;
% t3 = 0*(pi/180)   ; d3=0          ; a3=1         ;  alpha3 =    0;
% t4 = 0*(pi/180)   ; d4=0          ; a4=0         ;  alpha4 =   90;
% t5 = 0*(pi/180)   ; d5=3          ; a5=0         ;  alpha5 =  -90;
% t6 = 0*(pi/180)   ; d6=0          ; a6=0         ;  alpha6 =   90;
% t7 = 0*(pi/180)   ; d7=3          ; a7=0         ;  alpha7 =    0;
% A1~A7 �� 
Sum_T=So_Show_Matrix_from_DH( t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 )

% ���ں�� ��
So_Show_Jacobian_form_DH(t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );


% �ʱ������ ��������� ����. �ϴ� �̰� �� DH ��Ÿ�� ���� ������ -> �ϴ� �ʱⰪ����
Position_X =  Sum_T(1,4);
Position_Y =  Sum_T(2,4);
Position_Z =  Sum_T(3,4);

% �ʱ� ���� XYZ�࿡ ���� NOA
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
% disp('�ʱ� X,Y,Z, x��ȸ��, y��ȸ��, z��ȸ��   (��, ȸ���� z��, y��, x�� ������ ȸ��)')
% Position_Theta = [Position_X; Position_Y ; Position_Z ; Ini_Delta_X_N ; Ini_Delta_Y_O ;Ini_Delta_Z_A ]
% disp('////////////////////////////////////////////////////////////////////////////////////')
% disp('   ')
% disp('   ')

%% ������ǥ�� ���� ��.

World_RPY = So_World_ZYX( t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
CCCC=World_RPY(3)

disp('////////////////////////////////////////////////////////////////////////////////////')
disp('�ʱ� X,Y,Z, x��ȸ��, y��ȸ��, z��ȸ��   (��, ȸ���� z��, y��, x�� ������ ȸ��)')
Position_Theta = [Position_X; Position_Y ; Position_Z ; World_RPY*(pi/180) ]
disp('////////////////////////////////////////////////////////////////////////////////////')
disp('   ')
disp('   ')




%% �����ں���� ���� dP ���
% dP = [dx; dy; dz; N;O;A]
section = 1000;
%%%%%%%%%%%%%%%%%%%% ���⸸ �Է��� @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
Object_PTheta = [300 ; 200 ; 100 ; (pi/180)*0 ; (pi/180)*0 ; (pi/180)*90]; % ��ǥ�� �Է�   NOA�� Rad���� �������.

%%%%%%%%%%%%%%%%%%%%���������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dP = (Object_PTheta - Position_Theta)/section  %�����ġ - �ʱ���ġ �� section��ŭ ����
Theta([1 2 3 4 5 6 7],1) = [t1;t2;t3;t4;t5;t6;t7];

%% �����ں���� �̿��� Theta ���
Anti_CN = 0;

for i=1:section
             Inv_Jaco = So_Inv_Jacobian_for_Cal(Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i),Theta(7,i) ,d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7);
             
            
%              if cond(Inv_Jaco) < 100
% %                  disp('====================================================')
%                 i             ;                                  %i��°
%                 Theta_Check = Theta([1 2 3 4 5 6 7],i) *(180/pi);  %����
%                 Condition_Number = cond(Inv_Jaco)     ;          %���Ǽ�
%                 DH_ing = So_Show_Matrix_from_DH(Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i), Theta(7,i) , d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
%                 [XYZ_Under,RPY_Under] = So_Position_and_Theta_Cal( Theta(1,i),Theta(2,i),Theta(3,i),Theta(4,i),Theta(5,i),Theta(6,i), Theta(7,i) , d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );
%                 XYZRPY([1 2 3 4 5 6],1) = [XYZ_Under([1 2 3],4);RPY_Under] ;
%                 Inv_Jaco_Under = Inv_Jaco ;                     %�����ں��
%                 Before_Inv_Jaco = Inv_Jaco ;                    %���Ǽ� ��������� ������ ���ں��
%                 Before_Theta = Theta([1 2 3 4 5 6 7],i);          %���Ǽ� ��������� ������ ���� ����
%                 Show_Before_Theta = Before_Theta *(180/pi)   ;    %������ ���� ��Ÿ�� ��� 
%                 
%                 dP_Times = 1;
                Theta([1 2 3 4 5 6 7],i+1) = Theta([1 2 3 4 5 6 7],i)+ (Inv_Jaco*dP);
%                 
% %                 disp('====================================================')
%              else
% %                  disp(' ')
% %                  disp('####################################################')
% %                  disp(' ')
% %                  disp('Condition-Number ���ذ� �ʰ�')
%                  i             ;                                      %i��°
%                 Theta_Check_Over = Theta([1 2 3 4 5 6 7],i) *(180/pi) ; %����
%                 Condition_Number_Over = cond(Inv_Jaco)    ;           %���Ǽ�
%                 Inv_Jaco_Over = Inv_Jaco;                            %�����ں��
%                 Before_Inv_Jaco_Over = Before_Inv_Jaco;              %���Ǽ� ������� ������ ���ں��
%                 Before_Theta_Over = Before_Theta;                    %���Ǽ� �Ѿ �� �� ��Ÿ �������� �ҷ���
%                 Show_Before_Theta_Over = Before_Theta_Over *(180/pi); %���Ȱ��� ������
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



%���� ��Ÿ�� ���� ��׸��� ǥ��.

Rad_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1) ; Theta(7,section+1)]
disp('   ')
disp('=====================================================================')
disp('�����ں���� �̿��� ���� ��Ÿ1~��Ÿ6')
Deg_Theta = [Theta(1,section+1); Theta(2,section+1); Theta(3,section+1); Theta(4,section+1); Theta(5,section+1); Theta(6,section+1);Theta(7,section+1)] * (180/pi)
disp('=====================================================================')
disp('   ')

%% ������� �˻� ����
[Re_SUM_T,World_Rotate] = So_Position_and_Theta_Cal( Theta(1,section+1), Theta(2,section+1), Theta(3,section+1), Theta(4,section+1), Theta(5,section+1), Theta(6,section+1),Theta(7,section+1), d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,alpha7 );
Re_SUM_T
Position_R_X = Re_SUM_T(1,4);
Position_R_Y = Re_SUM_T(2,4);
Position_R_Z = Re_SUM_T(3,4);
disp('   ')
disp('   ')
disp('*********************************************************************************')
disp('������ ���� ��Ÿ1~��Ÿ6�� Forward Kinematics�� ���')
Position_Theta = [Position_R_X; Position_R_Y ; Position_R_Z ; World_Rotate ]
disp('*********************************************************************************')


%% �׷��� �׸���
%   ��Ÿ ��� ������


     for i = 1:section+1
      
              t1 = Theta(1,i);
              t2 = Theta(2,i);
              t3 = Theta(3,i);
              t4 = Theta(4,i);
              t5 = Theta(5,i);
              t6 = Theta(6,i);
              t7 = Theta(7,i);
                [Plot_A1, Plot_A2, Plot_A3, Plot_A4, Plot_A5, Plot_A6, Plot_A7] = So_Matrix_for_Cal(  t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );
              
                      A1 = Plot_A1    ;
                      A12 = Plot_A2;
                      A123 = Plot_A3;
                      A1234 = Plot_A4;
                      A12345 = Plot_A5;
                      A123456 = Plot_A6;
                      A1234567 = Plot_A7;
       

        Plot_X(i,2) = roundn(A1(1,4),-3); % 1�� X�� ��ġ
        Plot_Y(i,2) = roundn(A1(2,4),-3); % 1�� Y�� ��ġ
        Plot_Z(i,2) = roundn(A1(3,4),-3); % 1�� Z�� ��ġ
        Plot_X(i,3) = roundn(A12(1,4),-3); % 2�� X�� ��ġ
        Plot_Y(i,3) = roundn(A12(2,4),-3); % 2�� Y�� ��ġ
        Plot_Z(i,3) = roundn(A12(3,4),-3); % 2�� Z�� ��ġ
        Plot_X(i,4) = roundn(A123(1,4),-3); % 3�� X�� ��ġ
        Plot_Y(i,4) = roundn(A123(2,4),-3); % 3�� Y�� ��ġ
        Plot_Z(i,4) = roundn(A123(3,4),-3); % 3�� Z�� ��ġ   
        Plot_X(i,5) = roundn(A1234(1,4),-3); % 4�� X�� ��ġ
        Plot_Y(i,5) = roundn(A1234(2,4),-3); % 4�� Y�� ��ġ
        Plot_Z(i,5) = roundn(A1234(3,4),-3); % 4�� Z�� ��ġ 
        Plot_X(i,6) = roundn(A12345(1,4),-3); % 5�� X�� ��ġ
        Plot_Y(i,6) = roundn(A12345(2,4),-3); % 5�� Y�� ��ġ
        Plot_Z(i,6) = roundn(A12345(3,4),-3); % 5�� Z�� ��ġ    
        Plot_X(i,7) = roundn(A123456(1,4),-3) ;% 6�� X�� ��ġ
        Plot_Y(i,7) = roundn(A123456(2,4),-3); % 6�� Y�� ��ġ
        Plot_Z(i,7) = roundn(A123456(3,4),-3); % 6�� Z�� ��ġ   
        Plot_X(i,8) = roundn(A1234567(1,4),-3); % 7�� X�� ��ġ
        Plot_Y(i,8) = roundn(A1234567(2,4),-3); % 7�� Y�� ��ġ
        Plot_Z(i,8) = roundn(A1234567(3,4),-3); % 7�� Z�� ��ġ    
        
     end

        hold on
        Plot_Count = 0;
       for i=1: (section)/10 :section
            
                 if Plot_Count <= 1
                       plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'k-')
                       plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'ko-')
                       Plot_Count = Plot_Count +1;
                 elseif Plot_Count <= 3
                       plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'m-')
                       plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'mo-')
                       Plot_Count = Plot_Count +1;
                 elseif Plot_Count <= 5
                    plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'g-')
                    plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'go-')
                    Plot_Count = Plot_Count +1;
                 elseif Plot_Count <= 7
                    plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'c-')
                    plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'co-')
                    Plot_Count = Plot_Count +1  ;         
                 else 
                    plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'b-') 
                    plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'bo-')
                    Plot_Count = Plot_Count +1 ;
                 end
                 xlabel('X ��')
                 ylabel('Y ��')
                 zlabel('Z ��')
                 grid
        axis([-500 500 -500 500 -500 500])
       end
       
        plot3(Plot_X(section+1,:),Plot_Y(section+1,:), Plot_Z(i,:) , 'r-')
        plot3(Plot_X(section+1,8),Plot_Y(section+1,8), Plot_Z(i,8) , 'r-*')
       toc()
%% å�� �����ִ� ������� ��
% ������ ���� A1~A6������ ��Ŀ� ������ ��Ÿ�� ���� A123456 ������ ��ȯ�Ѵ�.
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

%% �˻�
% [All,RX,RY,RZ]=So_Position_and_Theta_Cal( Th1 ,Th2 ,Th3 ,Th4 ,Th5 ,Th6 , d1,d2,d3,d4,d5,d6, a1,a2,a3,a4,a5,a6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 )



