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

% UR �ʱⰪ �� ���
%  Ini_t = [(pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90  (pi/180)*90];
 Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0  (pi/180)*0];
% Goal_t = [(pi/180)*(33) (pi/180)*33 (pi/180)*(-41) (pi/180)*76 (pi/180)*(-9) (pi/180)*(-15)  (pi/180)*26];     
Goal_t = [(pi/180)*(-3.6313) (pi/180)*29.3335 (pi/180)*(16.9509) (pi/180)*75.8842 (pi/180)*(45.5186) (pi/180)*(-28.4714)  (pi/180)*15];

    d = [278 0 425 0 425 0 72];
    a = [0 0 0 0 0 0 0];
    alpha = [-90 90 -90 90 -90 90 0];
 History_Ave_Theta(:,1) = Ini_t*(180/pi);  

    
%% DH �� �ʱⰪ ����
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

% ����ġ
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
% 2D�϶� ��ֹ�
% Obstacle{1} = [3 3 0 ; 3 4 0; 4 4 0; 4 3 0; 3 3 0];       % 1�� ��ֹ�
% 
% Obs_Circle{1} = [3 4 0];
% Obs_Circle{2} = [6.5 2 0];
% Obs_Cir_Radius = [1 1];

% 7���϶� ��ֹ�
Obs_Circle{1} = [400 0 700];
% Obs_Circle{2} = [6.5 2 0];
Obs_Cir_Radius = [0];

obs_thrs = [5 5 5 5]  ;                         % ��ֹ� ��谪
% Transe_Obs= Obstacle{1}' ;                       % ���η� ��ǥ ǥ��
Bound_con = Obs_Cir_Radius*0.8    ;                   % ��谪�� ���������� ����
Bound_con_2 = Obs_Cir_Radius*3 ;
Try_Num = 800;
Limit_Value =1;


 [Ave_P_All,Ave_X,Ave_Y,Ave_Z]=So_Theta_to_Position( Ini_t )   
%% ��ֹ�

% 2D �簢��
% 	for i = 1:length(Obstacle)
% 		obs = Obstacle{i};
% 		patch(Obstacle{i}(:,1),Obstacle{i}(:,2),Obstacle{i}(:,3),'black');  % x,y �� �� ä��
%     end
    
% 3D ��
figure(1)
hold on
for i = 1 : length(Obs_Circle)
[Cir_x,Cir_y,Cir_z] = sphere;               %�ϴ� �� ����
surf((Obs_Cir_Radius(i)*0.8*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*0.8*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*0.8*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)      % ũ�� 3¥�� 5,4,0�� ����
colormap winter
grid off
end
view(45,20)

% figure(2)
% hold on
% for i = 1 : length(Obs_Circle)
% [Cir_x,Cir_y,Cir_z] = sphere;               %�ϴ� �� ����
% surf((Obs_Cir_Radius(i)*0.8*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*0.8*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*0.8*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)      % ũ�� 3¥�� 5,4,0�� ����
% colormap winter
% grid off
% end
%% ��ǥ��
% hold on
% quiver3(0,0,0,100,0,0, 'color','red', 'LineWidth', 2)
% quiver3(0,0,0,0,100,0, 'color','green', 'LineWidth', 2)
% quiver3(0,0,0,0,0,100, 'color','blue', 'LineWidth', 2)
view(45,20)  % 3D �̹��� ���� ���� ����
%%  ���
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
    [ P_All,X,Y,Z]=So_Theta_to_Position( Theta )  ;           %�ʱⰪ���� ���ϴ� ��Ÿ
    [ Goal_All,Goal_X,Goal_Y,Goal_Z]=So_Theta_to_Position( Goal_t );
    J=So_Jacobian( Theta );
    History_X(:,Repeat) = cell2mat(X(:,L_end));
    History_Y(:,Repeat) = cell2mat(Y(:,L_end));
    History_Z(:,Repeat) = cell2mat(Z(:,L_end));
    History_Theta(:,Repeat) = Theta*(180/pi);
    Mat_X = cell2mat(X);
    Mat_Y = cell2mat(Y);
    Mat_Z = cell2mat(Z);
    
    %���Theta �����
    
    if Repeat >= 2
        Ave_Theta=(Pre_Theta+Theta)./2 ;
        [Ave_P_All,Ave_X,Ave_Y,Ave_Z]=So_Theta_to_Position( Ave_Theta )  ; 
        History_Ave_Theta(:,Repeat) = Ave_Theta*(180/pi);
    end
    
    
    
    % Att ��
    for i = 1 : DOF
       for j = 1 : length(mid_point)
            F_att{i,j}= Coeff_att*(Goal_All{i,j}-P_All{i,j}) ;       % 2���̶� Z�� ������
            Line_att{i,j} = (Goal_All{i,j}-P_All{i,j})/norm((Goal_All{i,j}-P_All{i,j}));
            
            if ATT_POWER < norm(F_att{i,j})
                ATT_POWER = norm(F_att{i,j});
            end
       end
    end
    
    % Rep ��
    for i = 1 : DOF
           for j = 1 : length(mid_point)
%                F_rep{i,j}=[0,0,0];
               D_Vec{i,j}=[0;0;0];
               Temp_F_rep =[0;0;0];
               for Num_Obs = 1 : length(Obs_Circle)

                       
                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                   % ��ֹ��� �簢������ �����ؼ� ���� 2D ��ֹ� �ڵ�
                                   %                        Current_Dist_obs = [];
%                                                           Min_Dist_obs = [];
                            % % % % % % % % %         for ob_i = 1 : length( Transe_Obs-1)
                            % % % % % % % % %             Current_Dist_obs =   P_All{i,j}-Transe_Obs(:,ob_i); 
                            % % % % % % % % %             
                            % % % % % % % % %             if ob_i == 1 || (norm(Current_Dist_obs) < norm(Min_Dist_obs))
                            % % % % % % % % %                 D_Vec{i,j}= Current_Dist_obs;  % ��ֹ��� ������ �Ÿ��� ���Ͱ� xyz�� ��Ÿ��
                            % % % % % % % % %                 Min_Dist_obs = Current_Dist_obs;
                            % % % % % % % % %                 Min_obs_position{i,j} = Transe_Obs(:,ob_i);
                            % % % % % % % % %             end
                            % % % % % % % % %             
                            % % % % % % % % %         end
                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       

                             % ���� �� �Ÿ� ����.
                         for Cir_i = 1 : DOF
                            for Cir_j =1 : length(mid_point)
                                D_Vec{Cir_i,Cir_j,Num_Obs} = P_All{Cir_i,Cir_j}-Obs_Circle{Num_Obs}' ;
                            end
                         end


                        D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % �� �״�� �Ÿ� (������ġ-�߽�) - ������
%                           D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs});
                            if D_Norm{i,j,Num_Obs} <= Bound_con(Num_Obs)
                                
                                Part_F_rep{i,j,Num_Obs} = Coeff_rep(i+j-1)*( (1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) ) ).*(1/D_Norm{i,j,Num_Obs}).^3 .* (D_Vec{i,j,Num_Obs});
                                Temp_F_rep = Temp_F_rep + 0*Coeff_rep(i+j-1)*( (1/D_Norm{i,j,Num_Obs}) - ( 1/Bound_con(Num_Obs) ) ).*(1/D_Norm{i,j,Num_Obs}).^3 .* (D_Vec{i,j,Num_Obs});
                                
                            else
                                Part_F_rep{i,j,Num_Obs} = [0;0;0];
                                Temp_F_rep = Temp_F_rep + [0;0;0];        % Temp_F�� �ǹ̴� ��ֹ��� 2�� �̻�������, �� ��ֹ� ��ΰ���� �ݷ��� ��
                            end
                            F_rep{i,j}=  0*Temp_F_rep;
                            
                            if REP_POWER < norm(F_rep{i,j})
                                REP_POWER = norm(F_rep{i,j});
                            end
                            
                            
               end
           end
    end

    
        % 2�� �� Rep ��  ATT �̿��ظ��簪
    for i = 1 : DOF
           for j = 1 : length(mid_point)
%                F_rep{i,j}=[0,0,0];
               D_Vec_2{i,j}=[0;0;0];
               Temp_F_rep_2 =[0;0;0];
               for Num_Obs = 1 : length(Obs_Circle)

                       
                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                   % ��ֹ��� �簢������ �����ؼ� ���� 2D ��ֹ� �ڵ�
                                   %                        Current_Dist_obs = [];
%                                                           Min_Dist_obs = [];
                            % % % % % % % % %         for ob_i = 1 : length( Transe_Obs-1)
                            % % % % % % % % %             Current_Dist_obs =   P_All{i,j}-Transe_Obs(:,ob_i); 
                            % % % % % % % % %             
                            % % % % % % % % %             if ob_i == 1 || (norm(Current_Dist_obs) < norm(Min_Dist_obs))
                            % % % % % % % % %                 D_Vec_2{i,j}= Current_Dist_obs;  % ��ֹ��� ������ �Ÿ��� ���Ͱ� xyz�� ��Ÿ��
                            % % % % % % % % %                 Min_Dist_obs = Current_Dist_obs;
                            % % % % % % % % %                 Min_obs_position{i,j} = Transe_Obs(:,ob_i);
                            % % % % % % % % %             end
                            % % % % % % % % %             
                            % % % % % % % % %         end
                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       

                             % ���� �� �Ÿ� ����.
                         for Cir_i = 1 : DOF
                            for Cir_j =1 : length(mid_point)
                                D_Vec_2{Cir_i,Cir_j,Num_Obs} = P_All{Cir_i,Cir_j}-Obs_Circle{Num_Obs}' ;
                            end
                         end
                         
                         Ch_REP_VEC{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
                         
                         Line_rep{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs}/norm(D_Vec_2{i,j,Num_Obs});

                         
                        D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % �� �״�� �Ÿ� (������ġ-�߽�) - ������
                        Bound_Circle(Num_Obs) = Bound_con(Num_Obs) + Obs_Cir_Radius(Num_Obs) ;             % ��輱1
                        Bound_Circle_2(Num_Obs) = Bound_con_2(Num_Obs) + Obs_Cir_Radius(Num_Obs);          % ��輱2
%                         D_Coff_2_Norm{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs}) 
%                         Distance_Coff_2(i,j) = D_Coff_2_Norm{i,j,Num_Obs}/Bound_con_2
                        
%                           D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs});

                                Line_Theta{i,j,Num_Obs} = acos(dot(Line_att{i,j},Line_rep{i,j,Num_Obs})/(norm(Line_att{i,j})*norm(Line_rep{i,j,Num_Obs}))) *(180/pi);
                        
                        
                            if D_Norm_2{i,j,Num_Obs} <= Bound_con_2(Num_Obs)
                                
                                
                                
                                if D_Norm_2{i,j,Num_Obs} < Bound_con(Num_Obs)
                                    Current_Coff_2{i,j,Num_Obs} = 0.5;
                                else
                                    Current_Coff_2{i,j,Num_Obs} = (( Bound_con_2(Num_Obs)-D_Norm_2{i,j,Num_Obs} )/( Bound_con_2(Num_Obs)-Bound_con(Num_Obs) ))/2;  %��輱 2���� �۾����� ���������� ���� 0~1 ����. 
                                end                                                                                                                  % 1���� ũ�ٴ¼Ҹ��� ���1���� �� �������ǹ�
                                Part_F_rep_2{i,j,Num_Obs} = 1.* norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
                                Temp_F_rep_2 = Temp_F_rep_2 + 10.*norm(F_att{i,j}) .*  D_Vec_2{i,j,Num_Obs} / norm(D_Vec_2{i,j,Num_Obs});
                            else
                                Current_Coff_2{i,j,Num_Obs} = [0];
                                Part_F_rep_2{i,j,Num_Obs} = [0;0;0];
                                Temp_F_rep_2 = Temp_F_rep_2 + [0;0;0];        % Temp_F�� �ǹ̴� ��ֹ��� 2�� �̻�������, �� ��ֹ� ��ΰ���� �ݷ��� ��
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
    
                                % Att�� ���̱�
            for i = 1 : DOF
                 for j = 1 : length(mid_point)
                     if min(min(cell2mat(D_Norm_2))) <= Bound_con_2(Num_Obs)
                        if Line_Theta{i,j,Num_Obs} >= 90  %�� ���������� ������ 90���� ũ�� -> ���������� �ƴ϶��. 
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
% %             % 2�� �� Rep �� -> �ǹ̰� ���� �ٿ������ Ŀ���µ� �ݷ��� ������ ��ó���� Ŀ��
% %     for i = 1 : DOF
% %            for j = 1 : length(mid_point)
% % %                F_rep{i,j}=[0,0,0];
% %                D_Vec_2{i,j}=[0;0;0];
% %                Temp_F_rep_2 =[0;0;0];
% %                for Num_Obs = 1 : length(Obs_Circle)
% % 
% %                        
% %                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %                                    % ��ֹ��� �簢������ �����ؼ� ���� 2D ��ֹ� �ڵ�
% %                                    %                        Current_Dist_obs = [];
% % %                                                           Min_Dist_obs = [];
% %                             % % % % % % % % %         for ob_i = 1 : length( Transe_Obs-1)
% %                             % % % % % % % % %             Current_Dist_obs =   P_All{i,j}-Transe_Obs(:,ob_i); 
% %                             % % % % % % % % %             
% %                             % % % % % % % % %             if ob_i == 1 || (norm(Current_Dist_obs) < norm(Min_Dist_obs))
% %                             % % % % % % % % %                 D_Vec_2{i,j}= Current_Dist_obs;  % ��ֹ��� ������ �Ÿ��� ���Ͱ� xyz�� ��Ÿ��
% %                             % % % % % % % % %                 Min_Dist_obs = Current_Dist_obs;
% %                             % % % % % % % % %                 Min_obs_position{i,j} = Transe_Obs(:,ob_i);
% %                             % % % % % % % % %             end
% %                             % % % % % % % % %             
% %                             % % % % % % % % %         end
% %                              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
% % 
% %                              % ���� �� �Ÿ� ����.
% %                          for Cir_i = 1 : DOF
% %                             for Cir_j =1 : length(mid_point)
% %                                 D_Vec_2{Cir_i,Cir_j,Num_Obs} = P_All{Cir_i,Cir_j}-Obs_Circle{Num_Obs}' ;
% %                             end
% %                          end
% %                          
% %                          Line_rep{i,j,Num_Obs} = D_Vec_2{i,j,Num_Obs}/norm(D_Vec_2{i,j,Num_Obs});
% % 
% %                          
% %                         D_Norm_2{i,j,Num_Obs} = norm(D_Vec_2{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % �� �״�� �Ÿ� (������ġ-�߽�) - ������
% %                         Bound_Circle(Num_Obs) = Bound_con(Num_Obs) + Obs_Cir_Radius(Num_Obs) ;             % ��輱1
% %                         Bound_Circle_2(Num_Obs) = Bound_con_2(Num_Obs) + Obs_Cir_Radius(Num_Obs);          % ��輱2
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
% %                                 Current_Coff_2{i,j,Num_Obs} = ( Bound_con_2(Num_Obs)-D_Norm_2{i,j,Num_Obs} )/( Bound_con_2(Num_Obs)-Bound_con(Num_Obs) );  %��輱 2���� �۾����� ���������� ���� 0~1 ����. 
% %                                                                                                                                                         % 1���� ũ�ٴ¼Ҹ��� ���1���� �� �������ǹ�
% %                                 
% %                                 Part_F_rep_2{i,j,Num_Obs} = Coeff_rep(i+j-1)*( (1/D_Norm_2{i,j,Num_Obs}) - ( 1/Bound_con_2(Num_Obs) ) ).*(1/D_Norm_2{i,j,Num_Obs}).^3 .* (D_Vec_2{i,j,Num_Obs});
% %                                 Temp_F_rep_2 = Temp_F_rep_2 + 1000000*Coeff_rep(i+j-1)*( (1/D_Norm_2{i,j,Num_Obs}) - ( 1/Bound_con_2(Num_Obs) ) ).*(1/D_Norm_2{i,j,Num_Obs}).^3 .* (D_Vec_2{i,j,Num_Obs});
% %                             else
% %                                 Part_F_rep_2{i,j,Num_Obs} = [0;0;0];
% %                                 Temp_F_rep_2 = Temp_F_rep_2 + [0;0;0];        % Temp_F�� �ǹ̴� ��ֹ��� 2�� �̻�������, �� ��ֹ� ��ΰ���� �ݷ��� ��
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
 
    
    %�߰� ����
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

        [Current_Diff_XYZ Pre_Current_Diff_XYZ];  % �̰� ���ؼ� ������ ������ ��ȣ�� �ٲ�°�
%         [Pre_Diff_Two_XYZ Diff_Two_XYZ] %��������� �� �ΰ��� ���� ���� ���� �������� ����
        % ��ȭ���� ��ȣ�� ��ȭ��, -2�� ���� ������츦 �������� ����?
        
        
        % ���ù̴ϸ� ��ġ ã��
        if Current_Diff_XYZ.*Pre_Current_Diff_XYZ <= 0
            Check_Switch_Timer = Check_Switch_Timer+5;
        else
            Check_Switch_Timer = 0;
        end
        if Check_Switch_Timer > 100
            Diff_Two_XYZ;  %�׳� �ñ��ؼ� ǥ��
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
    %������� ����ϰ� Local_Minimum_Timer�� 100�� ������ ���ù̴ϸؿ� �����ٰ� �� �� �ִ�.
    
        % ���ù̴ϸ��϶� ���� ����ġ ON,OFF ���
        if Local_Minimum_Timer == 100
            Parameter_Switch = 1;   
        end
    
    
        %���� �̴ϸ� 100 ������ �ݷ� ����
    if and(Parameter_Switch == 1,Cre_Plot==1)
        [Ex_RepX,Ex_RepY,Ex_RepZ] = sphere;               %�ϴ� �� ����
        surf( (Ex_RepX)+Average_LM(1) , (Ex_RepY)+Average_LM(2), (Ex_RepZ)+Average_LM(3),'FaceAlpha',0.2,'EdgeAlpha',0)      % ũ�� 3¥�� 5,4,0�� ����
    %     colormap winter
        grid off
        Cre_Plot = 0;  % Ȥ�� �� �ݺ�������� �� �ٸ� ������ġ
        Ex_Switch = 1;       %���ù̴ϸ� 100�Ǵ¼��� Ex���ټ� ����ġ ON
        Ex_Obs_Circle{Ex_Num_Obs} = [Average_LM(1) Average_LM(2) Average_LM(3)] 
        Ex_Num_Obs = Ex_Num_Obs+1;
        Ex_Bound_con = 1;
        Ex_Att_Progress_Timer=0;    % �߰��ݷ� �η��� ���� �����ϸ� �ݷº��͹����� �߻��� �� �����Ƿ� �η��� ���� �����ϱ����� Ÿ�̸� �ʱ�ȭ
    end
    
    
    % �߰� Att ��
    if Ex_Switch == 1
%         Local_Minimum_Timer = 0;   % Ex���ټ� ����ġ ON�Ǵ¼��� ���ù̴ϸ� 0 (���� ���ù̴ϸ� ����� ���� �ʱ�ȭ)
        for i = 1 : DOF
           for j = 1 : length(mid_point)
                Ex_F_att{i,j}= 10*Coeff_att*(-P_All{i,j}) ;       % 2���̶� Z�� ������
                              % Ex_Att ��-> ��ũ
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

                                 % ���� �� �Ÿ� ����.
                             for Cir_i = 1 : DOF
                                for Cir_j =1 : length(mid_point)
                                    Ex_D_Vec{Cir_i,Cir_j,Ex_Num_Obs} = P_All{Cir_i,Cir_j}-Ex_Obs_Circle{Ex_Num_Obs}' ;
                                end
                             end


                            Ex_D_Norm{i,j,Ex_Num_Obs} = norm(Ex_D_Vec{i,j,Ex_Num_Obs});       % �� �״�� �Ÿ� (������ġ-�߽�)
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

                                                 % Ex_Rep ��-> ��ũ
                                               Ex_Torque_rep{i,j} = J{i,j}'*Ex_F_rep{i,j};
                                               Ex_Sum_Torque_rep = Ex_Sum_Torque_rep+Ex_Torque_rep{i,j};

               end
            end
        end
        
    end
    
    % ���ù̴ϸ� ����� Ex_Att �� �ʱ�ȭ
    if Ex_Switch == 1
        for Num_Obs = 1 : length(Obs_Circle)
            for i = 1 : DOF
                for j =1 : length(mid_point)
                     D_Vec{i,j}=[0;0;0];
                     D_Vec{i,j,Num_Obs} = P_All{i,j}-Obs_Circle{Num_Obs}' ;
                     D_Norm{i,j,Num_Obs} = norm(D_Vec{i,j,Num_Obs})-Obs_Cir_Radius(Num_Obs);       % �� �״�� �Ÿ� (������ġ-�߽�) - ������
                end
            end
        end
        
    
        if length(Obs_Circle)> length(Ex_Obs_Circle)
            for Add_for_Ex = (length(Obs_Circle)-length(Ex_Obs_Circle)+1) : length(Obs_Circle)    %Ex ��ֹ� ������ �� �����ϱ� ������ ��ֹ����� �Է�
                                                                                                %������� ��ֹ�2��, ���ù̴ϸذ��� ��ֹ���1���� ������ D���� 2���� ������
                for i = 1 : DOF
                  for j =1 : length(mid_point)
                     Ex_D_Norm{i,j,Add_for_Ex} = 10000;       %�ؿ��� ��谪���� ū ��� �߰� Att,Rep ���� 0���� �ٲٴµ� ������ �����ʱ� ���� ����
                  end
                end
            end
        elseif length(Obs_Circle) < length(Ex_Obs_Circle)
            for Add_for_Obs = (length(Ex_Obs_Circle)-length(Obs_Circle)+1) : length(Obs_Circle)    %�̰Ŵ� �ݴ�. ������ֹ� ������ �� �����ϱ� ������ֹ� ������ŭ ������
                for i = 1 : DOF
                  for j =1 : length(mid_point)
                     D_Norm{i,j,Add_for_Obs} = 10000;       %�ؿ��� ��谪���� ū ��� �߰� Att,Rep ���� 0���� �ٲٴµ� ������ �����ʱ� ���� ����
                  end
                end
            end            
        end

        
        
        % ������ �ʿ��� ����
        if Ex_Att_Progress_Timer >= 51
            if and(D_Norm{DOF,L_end,Num_Obs} > Bound_con(Num_Obs) , Ex_D_Norm{DOF,L_end,Num_Obs} > 1)  %�������
               for i = 1 : DOF
                     for j = 1 : length(mid_point)
                           Ex_F_att{i,j}= [0;0;0] ;       % 2���̶� Z�� ������
                                          % Ex_Att ��-> ��ũ
                           Ex_Torque_att{i,j} = zeros(DOF,1);
                           Ex_Sum_Torque_att = zeros(DOF,1);
                     end
               end 
                       Ex_Switch = 0
            end
        end

    end


    
    
    
    %==================================================================================================================================================================================================================
    
    

    
    
    
        % ��-> ��ũ
    for i = 1 : DOF
       for j = 1 : length(mid_point)
               % Att ��-> ��ũ
           Torque_att{i,j} = J{i,j}'*(F_att{i,j});
           Sum_Torque_att = Sum_Torque_att+Torque_att{i,j};
           Torque_att_Reverse{i,j} = J{i,j}'*(F_att_Reverse{i,j})*Limit_Value;
           Sum_Torque_att_Reverse = Sum_Torque_att_Reverse+Torque_att_Reverse{i,j};
               % Rep ��-> ��ũ
                              
           
           
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

    % �񱳸� ���� att��
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

    
    % Att+rep �� (�ð�ȭ ���� ǥ���ϱ� ���� �������)
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


    %% �׷��� �κ�

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
% % % %     % ������ ���� ���
% % % %     for i = 1 : DOF
% % % %            if i == 1
% % % %     %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %�����ϳ���
% % % %                plot3([0 Goal_X{i,L_end}],[0 Goal_Y{i,L_end}],[0 Goal_Z{i,L_end}], 'LineWidth', 2,'color','r');     %�������� 
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
% % % %     %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %�����ϳ���
% % % %                RGraph_L{i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);     %�������� 
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
% % % % %           Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'c', 'LineWidth', 1);  %��� ǥ��
% % % % %           Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %����ǥ��
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
% % % %     % % 10������ ���� �׷����� ����� �������� ��� ������ ����� �츲
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
% % % %     % %     �̰� �ǽð����� ����°�
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

   %% �׷��� �κ�2(��հ����� �׷��Ⱥ�����)

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

    % ������ ���� ���
    for i = 1 : DOF
           if i == 1
    %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %�����ϳ���
               plot3([0 Goal_X{i,L_end}],[0 Goal_Y{i,L_end}],[0 Goal_Z{i,L_end}], 'LineWidth', 2,'color','r');     %�������� 
           else
    %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
               plot3([Goal_X{i-1,L_end} Goal_X{i,L_end}],[Goal_Y{i-1,L_end} Goal_Y{i,L_end}],[Goal_Z{i-1,L_end} Goal_Z{i,L_end}], 'LineWidth', 2,'color','r') ;
           end
    end

    % plot3(Object(1),Object(2),Object(3),'ro')

    for i = 1 : DOF
           if i == 1
    %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %�����ϳ���
               Ave_RGraph_L{i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);     %�������� 
           else
    %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
               Ave_RGraph_L{i}=plot3([Ave_X{i-1,L_end} Ave_X{i,L_end}],[Ave_Y{i-1,L_end} Ave_Y{i,L_end}],[Ave_Z{i-1,L_end} Ave_Z{i,L_end}], 'LineWidth', 2) ;
           end
    end
                Eraser = Eraser +1;

                
        if mod(Repeat,10) == 1          %10������
            for i = 1 : DOF
            %    for j = 1 : length(mid_point)  % �̰� mid_point�� ����
                    j = length(mid_point);      % �̰� �� ��ũ ���� ����ǥ��
%                   Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'k', 'LineWidth', 1);  %��� ǥ��
%                   Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %����ǥ��
        %           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
        %           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
            %    end
            end
        end
        
                    for i = 1 : DOF
                       for j = 1 : length(mid_point)  % �̰� mid_point�� ����
%                             j = length(mid_point);  % �̰� �� ��ũ ���� ����ǥ��
        %                   Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'k', 'LineWidth', 1);  %��� ǥ��
%                           Disp_Vec{i,j}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %����ǥ��
%                             Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %����ǥ�� �� ��ũ ����
                %           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
                %           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
                       end
                     end

    if Repeat ==1
        pause(3)
    end

    pause(0.00002);

    % % 10������ ���� �׷����� ����� �������� ��� ������ ����� �츲
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

    % %     �̰� �ǽð����� ����°�
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
            
%             % ���Ͱ� mid_point �������� ����
%             for i = 1 : DOF
%                 for j = 1 : length(mid_point)
%                     delete(Disp_Vec{i,j});
%                 end
%             end
            
            
%             end
        end        
        
        
end


toc




