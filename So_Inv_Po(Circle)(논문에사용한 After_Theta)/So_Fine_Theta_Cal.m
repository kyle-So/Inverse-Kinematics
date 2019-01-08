function [ Get_Theta ] = So_Fine_Theta_Cal( X,Y,Z, Yaw, Picth, Roll,Ini_t  )
%SO_FINE_THETA_CAL Summary of this function goes here
%   Detailed explanation goes here

global d1 d2 d3 d4 d5 d6 d7 a1 a2 a3 a4 a5 a6 a7 alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7
%% DH 정의
syms t1 t2 t3 t4 t5 t6 t7
t1=0*(pi/180)    ; d1=278        ; a1=0         ;  alpha1 = -90;
t2=3*(pi/180)    ; d2=0          ; a2=0         ;  alpha2 =  90;
t3=0*(pi/180)    ; d3=425        ; a3=0         ;  alpha3 = -90;
t4=3*(pi/180)    ; d4=0          ; a4=0         ;  alpha4 =  90;
t5=0*(pi/180)    ; d5=425        ; a5=0         ;  alpha5 = -90;
t6=0*(pi/180)    ; d6=0          ; a6=0         ;  alpha6 =  90;
t7=0*(pi/180)    ; d7=72         ; a7=0         ;  alpha7 =   0;

t1 = Ini_t(1);
t2 = Ini_t(2);
t3 = Ini_t(3);
t4 = Ini_t(4);
t5 = Ini_t(5);
t6 = Ini_t(6);
t7 = Ini_t(7);

% t1=90*(pi/180)    ; d1=278        ; a1=0         ;  alpha1 = -90;
% t2=90*(pi/180)    ; d2=0          ; a2=0         ;  alpha2 =  90;
% t3=90*(pi/180)    ; d3=425        ; a3=0         ;  alpha3 = -90;
% t4=90*(pi/180)    ; d4=0          ; a4=0         ;  alpha4 =  90;
% t5=90*(pi/180)    ; d5=425        ; a5=0         ;  alpha5 = -90;
% t6=90*(pi/180)    ; d6=0          ; a6=0         ;  alpha6 =  90;
% t7=90*(pi/180)    ; d7=72         ; a7=0         ;  alpha7 =   0;

% (X,Y,Z,0,0,롤) 계산
% 1. 함수(X,Y,Z,0,0,롤) -> X,Y,Z,롤 까지계산한 세타1~세타7 나옴
[ThetaZ1,ThetaZ2,ThetaZ3,ThetaZ4,ThetaZ5,ThetaZ6,ThetaZ7] = So_Rotate_Z(X,Y,Z,Roll,t1,t2,t3,t4,t5,t6,t7);
disp('롤 맞춘 처음 세타값')
disp('RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR')
Theta_Roll = [ThetaZ1;ThetaZ2;ThetaZ3;ThetaZ4;ThetaZ5;ThetaZ6;ThetaZ7];
disp('RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR')
% 2, 위에서 계산한 세타1~7을 기본값으로 피치 돌림.
[ThetaY1,ThetaY2,ThetaY3,ThetaY4,ThetaY5,ThetaY6,ThetaY7] = So_Rotate_Y(X,Y,Z,Picth,ThetaZ1,ThetaZ2,ThetaZ3,ThetaZ4,ThetaZ5,ThetaZ6,ThetaZ7);
disp('요 맞춘 처음 세타값')
disp('YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY')
Theta_Yaw = [ThetaY1;ThetaY2;ThetaY3;ThetaY4;ThetaY5;ThetaY6;ThetaY7];
disp('YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY')
% % 3. 위에서 계산한 세타를 가지고 요 돌림
% So_Rotate_X(X,Y,Z,Yaw)

disp('마지막 세타값')
disp('====================================================================')
[Theta1,Theta2,Theta3,Theta4,Theta5,Theta6,Theta7] = So_Rotate_X(X,Y,Z,Yaw,ThetaY1,ThetaY2,ThetaY3,ThetaY4,ThetaY5,ThetaY6,ThetaY7);
Get_Theta_Deg = [Theta1,Theta2,Theta3,Theta4,Theta5,Theta6,Theta7]*(180/pi)
Get_Theta = [Theta1,Theta2,Theta3,Theta4,Theta5,Theta6,Theta7];
disp('====================================================================')
%% 그래프 그리기
% % % %   세타 모양 나오기
% % % figure(1)
% % %                  xlabel('X axis')
% % %                  ylabel('Y axis')
% % %                  zlabel('Z axis')
% % %                  view(45,20)
% % % 
% % % Plot_Theta([1 2 3 4 5 6 7],1) = [t1; t2; t3; t4; t5; t6; t7]*(pi/180);
% % % Plot_Theta_Rad([1 2 3 4 5 6 7],1) = Plot_Theta([1 2 3 4 5 6 7],1) * (pi/180);
% % % Return_Theta = [Theta1;Theta2;Theta3;Theta4;Theta5;Theta6;Theta7]*(180/pi)
% % % d_Theta = ( Return_Theta - Plot_Theta([1 2 3 4 5 6 7],1) )/10;
% % % 
% % %         for i = 2: 11
% % %             Plot_Theta([1 2 3 4 5 6 7],i) = Plot_Theta([1 2 3 4 5 6 7],i-1)+d_Theta ;
% % %             Plot_Theta_Rad([1 2 3 4 5 6 7],i) = Plot_Theta([1 2 3 4 5 6 7],i) * (pi/180);
% % %         end
% % %         
% % % 
% % % 
% % % 
% % %      for i = 1:11
% % %       
% % %               t1 = Plot_Theta_Rad(1,i);
% % %               t2 = Plot_Theta_Rad(2,i);
% % %               t3 = Plot_Theta_Rad(3,i);
% % %               t4 = Plot_Theta_Rad(4,i);
% % %               t5 = Plot_Theta_Rad(5,i);
% % %               t6 = Plot_Theta_Rad(6,i);
% % %               t7 = Plot_Theta_Rad(7,i);
% % %                 [Plot_A1, Plot_A2, Plot_A3, Plot_A4, Plot_A5, Plot_A6, Plot_A7] = So_Matrix_for_Cal(  t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );
% % %               
% % %                       A1 = Plot_A1    ;
% % %                       A12 = Plot_A2;
% % %                       A123 = Plot_A3;
% % %                       A1234 = Plot_A4;
% % %                       A12345 = Plot_A5;
% % %                       A123456 = Plot_A6;
% % %                       A1234567 = Plot_A7;
% % %        
% % % 
% % %         Plot_X(i,2) = roundn(A1(1,4),-3); % 1축 X축 위치
% % %         Plot_Y(i,2) = roundn(A1(2,4),-3); % 1축 Y축 위치
% % %         Plot_Z(i,2) = roundn(A1(3,4),-3); % 1축 Z축 위치
% % %         Plot_X(i,3) = roundn(A12(1,4),-3); % 2축 X축 위치
% % %         Plot_Y(i,3) = roundn(A12(2,4),-3); % 2축 Y축 위치
% % %         Plot_Z(i,3) = roundn(A12(3,4),-3); % 2축 Z축 위치
% % %         Plot_X(i,4) = roundn(A123(1,4),-3); % 3축 X축 위치
% % %         Plot_Y(i,4) = roundn(A123(2,4),-3); % 3축 Y축 위치
% % %         Plot_Z(i,4) = roundn(A123(3,4),-3); % 3축 Z축 위치   
% % %         Plot_X(i,5) = roundn(A1234(1,4),-3); % 4축 X축 위치
% % %         Plot_Y(i,5) = roundn(A1234(2,4),-3); % 4축 Y축 위치
% % %         Plot_Z(i,5) = roundn(A1234(3,4),-3); % 4축 Z축 위치 
% % %         Plot_X(i,6) = roundn(A12345(1,4),-3); % 5축 X축 위치
% % %         Plot_Y(i,6) = roundn(A12345(2,4),-3); % 5축 Y축 위치
% % %         Plot_Z(i,6) = roundn(A12345(3,4),-3); % 5축 Z축 위치    
% % %         Plot_X(i,7) = roundn(A123456(1,4),-3) ;% 6축 X축 위치
% % %         Plot_Y(i,7) = roundn(A123456(2,4),-3); % 6축 Y축 위치
% % %         Plot_Z(i,7) = roundn(A123456(3,4),-3); % 6축 Z축 위치   
% % %         Plot_X(i,8) = roundn(A1234567(1,4),-3); % 7축 X축 위치
% % %         Plot_Y(i,8) = roundn(A1234567(2,4),-3); % 7축 Y축 위치
% % %         Plot_Z(i,8) = roundn(A1234567(3,4),-3); % 7축 Z축 위치    
% % %         
% % %      end
% % % 
% % %         hold on
% % %         Plot_Count = 0;
% % %        for i=1: 10
% % % %             pause(1)
% % %                  if Plot_Count <= 1
% % %                        plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'k-')
% % %                        plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'ko-')
% % %                        Plot_Count = Plot_Count +1;
% % %                  elseif Plot_Count <= 3
% % %                        plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'m-')
% % %                        plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'mo-')
% % %                        Plot_Count = Plot_Count +1;
% % %                  elseif Plot_Count <= 5
% % %                     plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'g-')
% % %                     plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'go-')
% % %                     Plot_Count = Plot_Count +1;
% % %                  elseif Plot_Count <= 7
% % %                     plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'c-')
% % %                     plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'co-')
% % %                     Plot_Count = Plot_Count +1  ;         
% % %                  else 
% % %                     plot3(Plot_X(i,:),Plot_Y(i,:),Plot_Z(i,:),'b-') 
% % %                     plot3(Plot_X(i,8),Plot_Y(i,8),Plot_Z(i,8),'bo-')
% % %                     Plot_Count = Plot_Count +1 ;
% % %                  end
% % % 
% % % %                  grid
% % %         axis([-600 600 -600 600 0 1200])
% % %        end
% % %        
% % %         plot3(Plot_X(11,:),Plot_Y(11,:), Plot_Z(11,:) , 'r-')
% % %         plot3(Plot_X(11,8),Plot_Y(11,8), Plot_Z(11,8) , 'r*-')

end

