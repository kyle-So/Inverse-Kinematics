clc
% % clf
[DOF,Number]=size(History_Ave_Theta);
    t = 0.015;
% t=1 ;
    Time = 0 : t : t*(Number-1);
    Time_Vel = 0 : t : t*(Number-2) ;
    Time_Accel = 0 : t : t*(Number-3);
    for i = 2 : Number 
        History_Ave_Vel(:,i-1) = (History_Ave_Theta(:,i)-History_Ave_Theta(:,i-1))./t;
    end
    
    for i = 2 : Number-1 
        History_Ave_Accel(:,i-1) = (History_Ave_Vel(:,i)-History_Ave_Vel(:,i-1))./t;
    end
    

    
    %% 기존 출력
figure(2)
% title('Position')
    xlabel('time(s)')
    ylabel('Position(Deg)')
hold on
plot(Time,History_Ave_Theta(:,:))
%     plot(History_Ave_Theta')

figure(3)
% title('Velocity')
    xlabel('time(s)')
    ylabel('Angular Velocity(Deg/s)')
hold on
plot(Time_Vel,History_Ave_Vel(:,:))
% plot(History_Ave_Vel')
% % % % 
% % % % figure(4)
% % % % % title('Velocity')
% % % %     ylim([-200 200]);
% % % %     xlabel('time')
% % % %     ylabel('Angular Acceleration(Deg/s^2)')
% % % % hold on
% % % % plot(Time_Accel,History_Ave_Accel(:,:))
% % % % % plot(History_Ave_Vel')

% % 단위 환산 안함
% figure(2)
% % title('Position')
%     xlabel('time')
%     ylabel('Position')
% hold on
% % plot(Time,History_Ave_Theta(:,:))
%     plot(History_Ave_Theta')
% 
% figure(3)
% % title('Velocity')
%     xlabel('time')
%     ylabel('Angular Velocity(Deg/s)')
% hold on
% % plot(Time_Vel,History_Ave_Vel(:,:))
% plot(History_Ave_Vel')
% 
% figure(4)
% % title('Velocity')
%     ylim([-200 200]);
%     xlabel('time')
%     ylabel('Angular Acceleration(Deg/s^2)')
% hold on
% % plot(Time_Accel,History_Ave_Accel(:,:))
% plot(History_Ave_Accel')



%% 2차 궤적계획
% dfsdf = History_Ave_Theta(:,1)
% fefsfd = History_Ave_Theta(:,Final_Repeat)
%  unit_t = 0.0055
unit_t = 0.015;
i =1 ;


        Cubic_Theta = History_Ave_Theta;  % 일단 Cubic_Theta에 원래 평균각도를 집어넣고
        Cubic_Angular_Vel = History_Ave_Vel;
        
    for i = 1 : DOF

        for tt = 0 : Final_Repeat-1
            TTime(tt+1) = unit_t * tt;
        end
        
                                         
         % k를 이용한 for문을 이용해 포텐셜 아닐경우 속도를 보상해줌
        for k = 1 :2: Change_Jaco_Potential

            Qo = History_Ave_Theta(:,Record_Repeat(k))   ;         %데이터 값은 1부터 시작
            Qf = History_Ave_Theta(:,Record_Repeat(k+1));

            to = unit_t*(Record_Repeat(k)-1)  ;                         %시간은 0부터 시작...
            tf = unit_t*(Record_Repeat(k+1)-1);

        
        
        % 위치 그래프
            if k == 1
                for j = 1 : (Record_Repeat(k+1))       % 시간구역은 0 ~ n-1 까지지만 데이터는 1~n까지다
                     t = unit_t * (j-1);
                     % 2차함수로 그려지는 각도만들기. theta = Q, time = t  알파벳 o = 처음값 의미
                     Cubic_Theta(i,j) = Qo(i) + ( 3 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to).^2 - ( 2 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^3 ;
                end
            else
                for j = (Record_Repeat(k)+1) : ( Record_Repeat(k+1) )
                    t = unit_t * (j-1);
                    Cubic_Theta(i,j) = Qo(i) + ( 3 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to).^2 - ( 2 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^3 ;            
                end
            end
         
                % 각속도 그래프
            if k == 1
                for j = 1 : (Record_Repeat(k+1))       % 시간구역은 0 ~ n-1 까지지만 데이터는 1~n까지다
                     t = unit_t * (j-1);
                     % 2차함수로 그려지는 각도만들기. theta = Q, time = t  알파벳 o = 처음값 의미
%                      Cubic_Theta(i,t+1) = Qo(i) + ( 3 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to).^2 - ( 2 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^3 ;
                     Cubic_Angular_Vel(i,j) =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to) - ( 6 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^2 ;

                end
            else
                for j = (Record_Repeat(k)+1) : ( Record_Repeat(k+1) )
                    t = unit_t * (j-1);
%                     Cubic_Theta(i,t+1) = Qo(i) + ( 3 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to).^2 - ( 2 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^3 ;            
                    Cubic_Angular_Vel(i,j) =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to) - ( 6 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^2 ;

                end
            end

        end
     end
    


figure(5)
    plot(TTime,Cubic_Theta)  % 각도 2차
        xlabel('time(s)')
    ylabel('Position(Deg)')
    
figure(6)
    plot(TTime,Cubic_Angular_Vel)  % 각도 2차
        xlabel('time(s)')
    ylabel('Angular Velocity(Deg/s)')
    axis([0 7 -90 90])
        
    

%              Cubic_Angular_Vel(i,t+1) =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to) - ( 6 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^2 ;

             
             
             
             
             %% 속도
%              figure(6)   
%              i=1
%         Qo(i) = 15           %데이터 값은 1부터 시작
%         Qf(i) = 75
% 
%         to = 0                          %시간은 0부터 시작...
%         tf = 3
% t = [0:0.01:3]
%              Cubic_Angular_Vel =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to) - ( 6 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^2 ;
% 
% plot(t,Cubic_Angular_Vel)  % 각도 2차


%% 가속도
%              figure(7)   
%              i=1
%         Qo(i) = 15           %데이터 값은 1부터 시작
%         Qf(i) = 75
% 
%         to = 0                          %시간은 0부터 시작...
%         tf = 3
% t = [0:0.01:3]
%              Cubic_Angular_Acc =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) - ( 12 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to) ;
% 
% plot(t,Cubic_Angular_Acc)  % 각도 2차
% 

%% 바뀐 부분만 출력

% % 
% %         for k = 1 : Change_Jaco_Potential+1
% % 
% %                 Change_Theta = History_Ave_Theta(:,Record_Repeat(k)) ;           %데이터 값은 1부터 시작
% %                 Print_Theta{k} = Change_Theta ;
% %         end
% % 
% % Last_Experi = cell2mat(Print_Theta)' ;



%% Cubic_Theta를 이용한 재 시뮬레이션
% % figure(1)
% % for i = 1 : length(Obs_Circle)
% % [Cir_x,Cir_y,Cir_z] = sphere;               %일단 구 생성
% % surf((Obs_Cir_Radius(i)*Display_Coef*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*Display_Coef*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*Display_Coef*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)    % 크기 3짜리 5,4,0에 생성
% % colormap winter
% % grid off
% % end
% %     for i = 1 : Final_Repeat
% % %         i = 1
% %         C_Theta = Cubic_Theta(:,i) * (pi/180)
% %         [ Cubic_All,Cubic_X,Cubic_Y,Cubic_Z]=So_Theta_to_Position( C_Theta );
% %         So_Plot_Origin(Cubic_X,Cubic_Y,Cubic_Z, Goal_X,Goal_Y,Goal_Z, F_Sum, i, Eraser )
% %     end



