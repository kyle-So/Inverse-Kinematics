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
    

    
    %% ���� ���
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

% % ���� ȯ�� ����
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



%% 2�� ������ȹ
% dfsdf = History_Ave_Theta(:,1)
% fefsfd = History_Ave_Theta(:,Final_Repeat)
%  unit_t = 0.0055
unit_t = 0.015;
i =1 ;


        Cubic_Theta = History_Ave_Theta;  % �ϴ� Cubic_Theta�� ���� ��հ����� ����ְ�
        Cubic_Angular_Vel = History_Ave_Vel;
        
    for i = 1 : DOF

        for tt = 0 : Final_Repeat-1
            TTime(tt+1) = unit_t * tt;
        end
        
                                         
         % k�� �̿��� for���� �̿��� ���ټ� �ƴҰ�� �ӵ��� ��������
        for k = 1 :2: Change_Jaco_Potential

            Qo = History_Ave_Theta(:,Record_Repeat(k))   ;         %������ ���� 1���� ����
            Qf = History_Ave_Theta(:,Record_Repeat(k+1));

            to = unit_t*(Record_Repeat(k)-1)  ;                         %�ð��� 0���� ����...
            tf = unit_t*(Record_Repeat(k+1)-1);

        
        
        % ��ġ �׷���
            if k == 1
                for j = 1 : (Record_Repeat(k+1))       % �ð������� 0 ~ n-1 �������� �����ʹ� 1~n������
                     t = unit_t * (j-1);
                     % 2���Լ��� �׷����� ���������. theta = Q, time = t  ���ĺ� o = ó���� �ǹ�
                     Cubic_Theta(i,j) = Qo(i) + ( 3 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to).^2 - ( 2 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^3 ;
                end
            else
                for j = (Record_Repeat(k)+1) : ( Record_Repeat(k+1) )
                    t = unit_t * (j-1);
                    Cubic_Theta(i,j) = Qo(i) + ( 3 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to).^2 - ( 2 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^3 ;            
                end
            end
         
                % ���ӵ� �׷���
            if k == 1
                for j = 1 : (Record_Repeat(k+1))       % �ð������� 0 ~ n-1 �������� �����ʹ� 1~n������
                     t = unit_t * (j-1);
                     % 2���Լ��� �׷����� ���������. theta = Q, time = t  ���ĺ� o = ó���� �ǹ�
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
    plot(TTime,Cubic_Theta)  % ���� 2��
        xlabel('time(s)')
    ylabel('Position(Deg)')
    
figure(6)
    plot(TTime,Cubic_Angular_Vel)  % ���� 2��
        xlabel('time(s)')
    ylabel('Angular Velocity(Deg/s)')
    axis([0 7 -90 90])
        
    

%              Cubic_Angular_Vel(i,t+1) =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to) - ( 6 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^2 ;

             
             
             
             
             %% �ӵ�
%              figure(6)   
%              i=1
%         Qo(i) = 15           %������ ���� 1���� ����
%         Qf(i) = 75
% 
%         to = 0                          %�ð��� 0���� ����...
%         tf = 3
% t = [0:0.01:3]
%              Cubic_Angular_Vel =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) * (t-to) - ( 6 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to).^2 ;
% 
% plot(t,Cubic_Angular_Vel)  % ���� 2��


%% ���ӵ�
%              figure(7)   
%              i=1
%         Qo(i) = 15           %������ ���� 1���� ����
%         Qf(i) = 75
% 
%         to = 0                          %�ð��� 0���� ����...
%         tf = 3
% t = [0:0.01:3]
%              Cubic_Angular_Acc =  ( 6 / (tf-to).^2 ) * (Qf(i)-Qo(i)) - ( 12 / (tf-to).^3 ) * (Qf(i)-Qo(i)) * (t - to) ;
% 
% plot(t,Cubic_Angular_Acc)  % ���� 2��
% 

%% �ٲ� �κи� ���

% % 
% %         for k = 1 : Change_Jaco_Potential+1
% % 
% %                 Change_Theta = History_Ave_Theta(:,Record_Repeat(k)) ;           %������ ���� 1���� ����
% %                 Print_Theta{k} = Change_Theta ;
% %         end
% % 
% % Last_Experi = cell2mat(Print_Theta)' ;



%% Cubic_Theta�� �̿��� �� �ùķ��̼�
% % figure(1)
% % for i = 1 : length(Obs_Circle)
% % [Cir_x,Cir_y,Cir_z] = sphere;               %�ϴ� �� ����
% % surf((Obs_Cir_Radius(i)*Display_Coef*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*Display_Coef*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*Display_Coef*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)    % ũ�� 3¥�� 5,4,0�� ����
% % colormap winter
% % grid off
% % end
% %     for i = 1 : Final_Repeat
% % %         i = 1
% %         C_Theta = Cubic_Theta(:,i) * (pi/180)
% %         [ Cubic_All,Cubic_X,Cubic_Y,Cubic_Z]=So_Theta_to_Position( C_Theta );
% %         So_Plot_Origin(Cubic_X,Cubic_Y,Cubic_Z, Goal_X,Goal_Y,Goal_Z, F_Sum, i, Eraser )
% %     end



