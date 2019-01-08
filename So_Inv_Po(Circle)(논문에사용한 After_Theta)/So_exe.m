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

% % UR �ʱⰪ �� ���
% %  Ini_t = [(pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90  (pi/180)*90];
%  Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0  (pi/180)*0];
% Goal_t = [(pi/180)*(33) (pi/180)*33 (pi/180)*(-41) (pi/180)*76 (pi/180)*(-9) (pi/180)*(-15)  (pi/180)*26];     
% % Goal_t = [(pi/180)*(-3.6313) (pi/180)*29.3335 (pi/180)*(16.9509) (pi/180)*75.8842 (pi/180)*(45.5186) (pi/180)*(-28.4714)  (pi/180)*15];
% 
%     d = [278 0 425 0 425 0 72];
%     a = [0 0 0 0 0 0 0];
%     alpha = [-90 90 -90 90 -90 90 0];
%  History_Ave_Theta(:,1) = Ini_t*(180/pi);  

% �б� �κ� �����
%  Ini_t = [(pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90  (pi/180)*90];
%  Ini_t = [(pi/180)*(-73.263) (pi/180)*(68.5316) (pi/180)*(108.1991) (pi/180)*(109.287) (pi/180)*(214.9152) (pi/180)*(42.131)  (pi/180)*(81.4465)];
% Goal_t = [(pi/180)*(-38.4599) (pi/180)*(86.8663) (pi/180)*(120.815) (pi/180)*(109.2882) (pi/180)*(200.4894) (pi/180)*(76.5099)  (pi/180)*(65.823)];     
% Goal_t = [(pi/180)*(-3.6313) (pi/180)*29.3335 (pi/180)*(16.9509) (pi/180)*75.8842 (pi/180)*(45.5186) (pi/180)*(-28.4714)  (pi/180)*15];

% [ Ini_t ] = So_Fine_Theta_Cal(  550, -200 ,650 , 0 , 45 , -45 )   % �ʱ���ġ
% Ini_t = [-1.0699    1.5140    2.3533    1.3448    2.8469    0.1707    0.7697] ;  % ���迡����
% [ Goal_t ] = So_Fine_Theta_Cal( 550, 300 ,650 , 0 , 45 , -45 )  % ��ǥ��ġ
% Goal_t = [0.1419    1.3750    2.1392    1.1397    2.4698    0.9998    0.4809] ;  % ���迡����

% �� 3�� ���谪
% Ini_t = [-0.2493    0.4194   -0.4166    1.3447   -0.3343   -1.1174   -0.1171];
% Goal_t = [0.6641    0.5357    0.0191    1.1398    0.5201   -1.0789   -1.5466];

%�̰� �׳� �غ��°�
Ini_t = [-0.1696    0.3759   -0.5518    1.3056   -1.2812   -0.7048    1.4713];
Goal_t = [1.3375    0.8273   -1.3972    1.0976   -0.0953    0.2908    0.1438];

    d = [278 0 425 0 425 0 72];
    a = [0 0 0 0 0 0 0];
    alpha = [-90 90 -90 90 -90 90 0];
 History_Ave_Theta(:,1) = Ini_t*(180/pi);  
Obs_Circle{1} = [550 150 580];
% Obs_Circle{2} = [-200 500 0];
Obs_Cir_Radius = [100 0];

Change_Jaco_Potential =  0 ;  %��� ��ȭ�ߴ��� Ȯ�� ����ġ �ʱⰪ 0
SW_Jacobian = 0;            % ���ں���� ����ġ
SW_Potential = 0;           % ���ټ��� ����ġ
%% DH �� �ʱⰪ ����
% Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 ];
% Goal_t = [(pi/180)*90 (pi/180)*0 (pi/180)*(0) ];
% % Goal_t = [(pi/180)*90 (pi/180)*(-50) (pi/180)*(70) ];  
%     d = [0 0 0];
%     a = [300 200 200];
%     alpha = [0 0 0];
%  History_Ave_Theta(:,1) = Ini_t*(180/pi);  
mid_point = [0.5 1];
L_end = length(mid_point);
DOF = length(Ini_t);
Coeff_att = []; Coeff_rep = [];

Local_Minimum_Timer =0; Check_Switch_Timer = 0; Escape_Timer=0; Virtual_Att_Progress_Timer=0;
Min_Point = [0;0;0]; Divid = 0;
Virtual_Switch = 0; Virtual_Num_Obs = 1; Virtual_Coef_F_rep = 10; Cre_Plot =1; Parameter_Switch = 0; Virtual_Rep_Switch=0;
Virtual_Sum_Torque_att = zeros(DOF,1); Virtual_Sum_Torque_rep = zeros(DOF,1); Virtual_Bound_con = 1;
Pre_Pre_Theta=Ini_t;   Pre_Theta = Ini_t;
Coeff_Switch = 0;
for i = 1 : DOF
     for j = 1 : length(mid_point)
           Virtual_F_att{i,j}= [0;0;0] ;       %�������� �ʱⰪ ����
           Virtual_F_rep{i,j}= [0;0;0] ;
     end
end 

for i = 1: 10
    Ave_10_Theta(i,:) = Ini_t*0 ;
end

So_Coeff;  %����ġ �ҷ�����

Eraser = 1;
Theta = Ini_t;
Before_Check = zeros(DOF,L_end);
% 2D�϶� ��ֹ�
% Obstacle{1} = [3 3 0 ; 3 4 0; 4 4 0; 4 3 0; 3 3 0];       % 1�� ��ֹ�

% Obs_Circle{1} = [300 400 0];
% Obs_Circle{2} = [-200 500 0];
% Obs_Cir_Radius = [0 100];

% 7���϶� ��ֹ�
% Obs_Circle{1} = [400 0 800];
% Obs_Circle{2} = [6.5 2 0];
% Obs_Cir_Radius = [100];

obs_thrs = [5 5 5 5]  ;                         % ��ֹ� ��谪
% Transe_Obs= Obstacle{1}' ;                       % ���η� ��ǥ ǥ��
% Bound_con = [0.3 0.1]  
Bound_con = Obs_Cir_Radius*0.2        % ��谪�� ���������� ����
Bound_con_2 = Obs_Cir_Radius*1        %������ * 0.5 = �������� ���� ��谪
 
Try_Num = 2000;
Limit_Value =1;
Process_Potential = 0 ;

 [Ave_P_All,Ave_X,Ave_Y,Ave_Z]=So_Theta_to_Position( Ini_t ) ;  
%% ��ֹ�

% 2D �簢��
% 	for i = 1:length(Obstacle)
% 		obs = Obstacle{i};
% 		patch(Obstacle{i}(:,1),Obstacle{i}(:,2),Obstacle{i}(:,3),'black');  % x,y �� �� ä��
%     end
    
% 3D ��
figure(1)
Display_Coef = 1;         % �������� ���� ������� ���ϰ��ΰ�? 1�̸� ���� ������ 
hold on
for i = 1 : length(Obs_Circle)
[Cir_x,Cir_y,Cir_z] = sphere;               %�ϴ� �� ����
surf((Obs_Cir_Radius(i)*Display_Coef*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*Display_Coef*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*Display_Coef*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)    % ũ�� 3¥�� 5,4,0�� ����
colormap winter
grid off
end
% view(45,20)
view(114,32)
% view(84,16)
% % 3D �Ǹ���
% figure(1)
% Display_Cylinder_Coef = 1;         % �������� ���� ������� ���ϰ��ΰ�? 1�̸� ���� ������ 
% hold on
% for i = 1 : length(Obs_Circle)
% [Cyl_x,Cyl_y,Cyl_z] = cylinder;               %�ϴ� �� ����
% surf((Obs_Cir_Radius(i)*Display_Cylinder_Coef*Cyl_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*Display_Cylinder_Coef*Cyl_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*10*Display_Cylinder_Coef*Cyl_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)    % ũ�� 3¥�� 5,4,0�� ����
% colormap winter
% grid off
% end



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
% view(45,20)  % 3D �̹��� ���� ���� ����
%%  ���
for  Repeat = 1 : Try_Num
    % ��� �� �ʱⰪ �Է�

    Sum_Torque_att = zeros(DOF,1);
    Sum_Torque_att_Reverse = zeros(DOF,1);
    Sum_Torque_rep = zeros(DOF,1);
    Sum_Torque_rep_2 = zeros(DOF,1);
   Sum_Zero_F_repTorque_rep_2 = zeros(DOF,1);
    [ Ini_All,Ini_X,Ini_Y,Ini_Z]=So_Theta_to_Position( Ini_t );
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
    
    [ Switch_Obs,Norm_Q_to_Obs, Switch_F_Rep,Judge_Matrix_0, Judge_Matrix ] = So_Judge_Obstacle( Goal_All, P_All );       %������ġ�� ������ ��ġ ���±濡 ��ֹ��� �ִ°� ���°�?
%     Switch_Obs
    % ��ֹ��� ������ ���� ���ټ� �ʵ� ����� �ʿ� X
    % ���� ���� ������ ��ǥ ������ �����ϰ� ������
    
%     JM_0 = Judge_Matrix_0(:,:,1)
%     JM = Judge_Matrix(:,:,1)
%     Repeat;
    if and(max(Switch_Obs) == 0,Process_Potential == 0)   % ���±濡 ��ֹ��� ���ٸ�
        SW_Jacobian = SW_Jacobian + 1;
            if SW_Jacobian == 1
                Change_Jaco_Potential = Change_Jaco_Potential + 1;  % ��ȭ�� 1 ����
                SW_Potential = 0;   % ���ټȽ���ġ�� Off
                Record_Repeat(Change_Jaco_Potential) = Repeat;
            end
            
        [ Divide_Number,Judge_End ] = So_Distance_P_G( Ini_All,P_All,Goal_All );  % �Ÿ��� ���� ���
        
        Pre_Theta = Theta;
            Theta = Theta + ((Goal_t-Theta)/(3*Divide_Number));
            Theta_Deg = Theta * (180/pi);
            
                         for i = 1 : DOF
                           for j = 1 : length(mid_point)
                               F_Sum{i,j} = [0;0;0];
                           end
                         end   
            
    else
         SW_Potential = SW_Potential + 1;
            if SW_Potential == 1
                Change_Jaco_Potential = Change_Jaco_Potential + 1;  % ��ȭ�� 1 ����
                SW_Jacobian = 0;   % ���ټȽ���ġ�� Off
                Record_Repeat(Change_Jaco_Potential) = Repeat;
            end              
        
    % else��... ���⼭���ʹ� ���ټ� �ʵ�
    Process_Potential = 1 ;   % ���ټ� �ʵ� �����ߴٴ� ��. �̷��� ���ⱸ�� ������ �Ұ����� ���ټ��� �ʿ���ٰ� �ǴܵǸ� �� ������ 0�̵�.
                              % ���� 0�� �Ǿ� �� ���ΰ�?
        % ��¥ ��� ����.

        % Att���Ϳ� ��ֹ� ������ �Ÿ�
%         [ Norm_Q_to_Obs, Switch_F_Rep ] = So_Distance_Att_Obs( Goal_All, P_All )  % ���� ȣȯ ������ So_Judge_Obstacle�� �� ��� ����
%         Minimum_Vatt_Obs = min(min( cell2mat(Norm_Q_to_Obs) ))   ;              %�κ��ϰ� ��ǥ���������� �������� Att�� ��ֹ� Obs�� �ּҰŸ�
    %     Switch_F_Rep

        % 
% Switch_F_Rep

        % Att ��
        [ F_att,Unit_Vec_att  ] = So_F_att( Goal_All,P_All );

    %     FATT = cell2mat(F_att)

    % Rep ��
    % 0�� : �ٿ���� ���� ����(0.001?) �׳� ������ ��ü�� ������ -> �⺻ ���ټ� ô�»��
    % 1�� : �ٿ���� ���� ���� 0.3r? ���� �⺻ ���ټȻ��
    % 2�� : 0,1�� ô�¹߻��� ������ ô������ ���� �ް��� �ӵ� ���ӵ� ��ȭ�� �Ͼ -> �ⱸ����
    % �׷��⿡ �������� ����, ������ �Ͼ���� ������ ô�� ��� ���
    % ��¶��, ô�� �߰��Ҷ����� �ؿ� �׿��� ô�� �� �߰��ϱ� �������Ƿ� ���⼭ ô���� �ϳ��� ��ħ
    
      % 0�� Rep �� (�ٿ�������� �ٷ� ��ֹ�)
        [ Origin_Zero_F_rep ] = So_Zero_F_rep( Goal_All,P_All );
            for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle) %�ϴ� 0
                     F_rep_0{i,j,Num_Obs}=Origin_Zero_F_rep{i,j,Num_Obs};  % 0���� ����ġ�� ������ ������ �׻� ô���� ����  
                   end
               end
            end   % ������ ���� �浹 ���� �� ��ũ �� ���������� ��
  
        % 1�� Rep ��
        [ Origin_F_rep ] = So_F_rep( Goal_All,P_All );
            for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle)
                     F_rep_1{i,j,Num_Obs}=Origin_F_rep{i,j,Num_Obs}.*Switch_F_Rep(Num_Obs);  
                   end
               end
            end     % F_rep �� ��ũ �� ���������� ��

        % 2�� �� Rep ��  ATT �̿��ظ��簪
        [ Origin_F_rep_2 ]=So_Boundary2_Rep( Goal_All, P_All, F_att, Unit_Vec_att  );    
             for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle)
                     F_rep_2{i,j,Num_Obs}=Origin_F_rep_2{i,j,Num_Obs}.*Switch_F_Rep(Num_Obs);  % 0000000  
                   end
               end
             end   % 2�� �� F_rep �� ��ũ �� ���������� �� 
            
        %ô�� Rep, 0,1,2�� ��ġ��
            for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle)
                     F_rep{i,j,Num_Obs}=F_rep_0{i,j,Num_Obs}+F_rep_1{i,j,Num_Obs}+F_rep_2{i,j,Num_Obs}; % F_rep�� 012�� ��ģ��
                   end
               end
            end   


    % F2= cell2mat(F_rep_2)
        %Ȯ�ο�

    %     [cell2mat(Origin_F_rep) cell2mat(F_rep)];

        % ��ֹ��� ������ �η�,ô���� ����� �����ϰ�.
        if max(Switch_F_Rep) == 0         %��ֹ��� ������ ����ġ ų �غ�  
            Coeff_Switch = Coeff_Switch +1;
        else
             Coeff_Switch = 0;
        end

        if Coeff_Switch > 20            %����ġ�� 20�� ���̸� ��ֹ��� ����� ���ٰ� �Ǵ�.  
            So_Coeff_0               % ����� 1
            Process_Potential = 0  
        else
            So_Coeff
        end





        % i��°�� �� ���������� ����, i-1��°������ ����, �׸��� i,i-2������ ����(�̰Ŵ� �ڷΰ��� �ٽ� ������ ���� �� �� ���� ������ ���
    %     [Current_Diff_XYZ , Pre_Current_Diff_XYZ, Diff_Two_XYZ] = So_Vec_Direction(History_X,History_Y,History_Z,Repeat )

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
             end

          if Repeat >= 4 
            % ���ù̴ϸ� ��ġ ã��
            if Current_Diff_XYZ.*Pre_Current_Diff_XYZ <= 0     % i, i-1��°�� ������ �ݴ��̸�
%                 Check_Switch_Timer = Check_Switch_Timer+5;     % ���ù̴ϸ� �ǽɰ��ϱ� �ϴ� ī���� �ø�
Check_Switch_Timer = 0; % �ӽ� �� ���ù̴ϸؾ���
            else
                Check_Switch_Timer = 0;                        % �� �������̰� ���ٸ� �ϴ� ���� �̴ϸ� �ƴϴϱ� ī���� �ʱ�ȭ
            end

            if Check_Switch_Timer > 100
%                     if max(abs(Diff_Two_XYZ)) < 0.01      %�ι�° ����, �̵��Ÿ��� �ԽǷк��� ������
                                                % �� ���� �����Ͽ����� ����� �ʿ���
                       if max(abs(Diff_Two_XYZ)) < 1 % �ϴ� �������� 500 600 �Ѿ�Ƿ� 1����                        
                        Local_Minimum_Timer = Local_Minimum_Timer+5     % ������ �������� �̵��Ÿ����� �۰�, �ӵ��� ������ �ݴ�, 2������ �����ϹǷ� ���ù̴ϸ� ī���� 

                        if and(Local_Minimum_Timer >= 80, Local_Minimum_Timer <= 100)   % �� �κ� ���ڰ� ����ε�, ���ù̴ϸ��� �о������ ������ �ݷ� ������ ���� �Ұ��ΰ�... 
                            Divid = Divid +1;
                            Min_Point = Min_Point + [Mat_X(DOF,L_end);Mat_Y(DOF,L_end);Mat_Z(DOF,L_end)];
                            Average_LM=Min_Point/Divid;         %��¶��, Average x,y,z�� ����µ� ���� ������ �ʿ䰡�־��.
                        end

                    end
            end
         end


        %������� ����ϰ� Local_Minimum_Timer�� 100�� ������ ���ù̴ϸؿ� �����ٰ� �� �� �ִ�.

            % ���ù̴ϸ��϶� ���� ����ġ ON,OFF ���
            if Local_Minimum_Timer == 100    %���ù̴ϸ��� ��� ���������� �ϴ� 100�� ������ ����ġ �ø�
                Parameter_Switch = 1;        % �� �Ķ���ʹ� ������ �ݷ� �Ķ���͸� On
            end


            %���� �̴ϸ� 100 ������ �ݷ� ����
        if and(Parameter_Switch == 1,Cre_Plot==1)
            Vir_Obs_Cir_Radius = 100;
            [Virtual_RepX,Virtual_RepY,Virtual_RepZ] = sphere;               %�ϴ� �� ����
            surf( Vir_Obs_Cir_Radius*(Virtual_RepX)+Average_LM(1) , Vir_Obs_Cir_Radius*(Virtual_RepY)+Average_LM(2), Vir_Obs_Cir_Radius*(Virtual_RepZ)+Average_LM(3),'FaceAlpha',0.2,'EdgeAlpha',0)      % Average x,y,z
        %     colormap winter
            grid off
            Cre_Plot = 0;  % Ȥ�� �� �ݺ�������� �� �ٸ� ������ġ
            Virtual_Switch = 1;       %���ù̴ϸ� 100�Ǵ¼��� Ex���ټ� ����ġ ON
            Virtual_Obs_Circle{Virtual_Num_Obs} = [Average_LM(1) Average_LM(2) Average_LM(3)] ;
            Virtual_Num_Obs = Virtual_Num_Obs+1;  % ���� ������ �� ����ؼ� Index ��ȣ �ø�
            Virtual_Bound_con = 1;           % ������ �� ��谪
            Virtual_Att_Progress_Timer=0;    % �߰��ݷ� �η��� ���� �����ϸ� �ݷº��͹����� �߻��� �� �����Ƿ� �η��� ���� �����ϱ����� Ÿ�̸� �ʱ�ȭ
        end

      % ���� Att ��
        if Virtual_Switch == 1
    %         Local_Minimum_Timer = 0;   % Ex���ټ� ����ġ ON�Ǵ¼��� ���ù̴ϸ� 0 (���� ���ù̴ϸ� ����� ���� �ʱ�ȭ)
          [ Virtual_Sum_Torque_att,Virtual_F_att  ] = So_Virtual_Att( P_All,J );
            Virtual_Att_Progress_Timer = Virtual_Att_Progress_Timer + 1;      % ���� ô�� ������� ���� ���ð�
        end

        % Ÿ�̸�
        if Virtual_Att_Progress_Timer >= 50
            Virtual_Rep_Switch =1;          % ������ �η��� 50�� �ۿ������� ����ô�µ� ����
        end

        %���� ô�� Rep ��
        if Virtual_Rep_Switch == 1
           [ Virtual_Sum_Torque_rep, Virtual_F_rep ] = So_Virtual_Rep( P_All,J,Virtual_Obs_Circle );
        end

        % ������ ��ũ�� ���� �η� Vri_Att�� ��ֹ��� ������ �׻� �����η� ���ָ�???

        
%                    for Num_Obs = 1 : length(Obs_Circle)
                        if(Judge_Matrix(DOF,L_end,1)) ==0
                            Virtual_F_att{DOF,L_end} = [0;0;0];
                            Virtual_F_att{DOF,1} = [0;0;0];
                            Virtual_Sum_Torque_att(3,1) = 0;
                            Sum_Torque_att(3,1) = Sum_Torque_att(3,1) * 5;  %���� �����η��� 10�� �ִϱ� �����ϴ´�� 5�� �߰�... �ٵ� �̰� �����ؾ��ϳ�..?
                        end
%                    end

        
        
        
            % ô�� ���� ����. ���� ������ �Ÿ��� ��������ϱ� �ʿ���� �� ���� �ؾ���..
        if Virtual_Switch == 1

            if Virtual_Att_Progress_Timer >= 11
                if max(Switch_F_Rep) == 0  
    %                 B_Alpha = 0.5;
    %                 Bound_con = Obs_Cir_Radius*B_Alpha    ; % �ӽ÷� ��� ���߿��� �ٿ���� ������� ������ �ٲٵ��� ����
                   for i = 1 : DOF
                         for j = 1 : length(mid_point)
                               Virtual_F_att{i,j}= [0;0;0] ;       % 2���̶� Z�� ������
                                              % Virtual_Att ��-> ��ũ
                               Virtual_Torque_att{i,j} = zeros(DOF,1);
                               Virtual_Sum_Torque_att = zeros(DOF,1);
                         end
                   end 
                           Virtual_Switch = 0;Local_Minimum_Timer =0; Check_Switch_Timer = 0; Escape_Timer=0;Parameter_Switch = 0;Cre_Plot=1 ;Virtual_Att_Progress_Timer=0; 
                           Divid =0; Min_Point =0; Average_LM =[0;0;0];

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
    %            Torque_att_Reverse{i,j} = J{i,j}'*(F_att_Reverse{i,j})*Limit_Value;
    %            Sum_Torque_att_Reverse = Sum_Torque_att_Reverse+Torque_att_Reverse{i,j};

               for Num_Obs = 1 : length(Obs_Circle)
                   % Rep ��-> ��ũ
                   Torque_rep{i,j,Num_Obs} = J{i,j}'*F_rep{i,j,Num_Obs}*Limit_Value;
                   Sum_Torque_rep = Sum_Torque_rep+Torque_rep{i,j,Num_Obs};
%                    Torque_rep_2{i,j,Num_Obs} = J{i,j}'*F_rep_2{i,j,Num_Obs}*Limit_Value;
%                    Sum_Torque_rep_2 = Sum_Torque_rep_2+Torque_rep_2{i,j,Num_Obs};      
                   %
                   %0�� ��
%                    Torque_Zero_rep_2{i,j,Num_Obs} = J{i,j}'*Zero_F_rep{i,j,Num_Obs}*Limit_Value;
%                    Sum_Zero_F_repTorque_rep_2 = Sum_Zero_F_repTorque_rep_2+Torque_Zero_rep_2{i,j,Num_Obs};                

               end
           end
        end
%         cell2mat(Torque_att);
%         cell2mat(Torque_rep)
    % D_Norm
    % F_rep{2,2}

        % �񱳸� ���� att��
        Sum_Torque_att+Virtual_Sum_Torque_att+Sum_Torque_rep+Virtual_Sum_Torque_rep;
        Sum_Torque_rep;
        % Theta = Theta + Sum_Torque_att'*0.001
% Switch_F_Rep

        if Virtual_Switch == 0
            Sum_Torque = Sum_Torque_att+Sum_Torque_rep;
    %         Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2;
    %         Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2-Sum_Torque_att_Reverse;
    % Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2-Sum_Torque_att_Reverse;

            % Check_T([1 2 3 4],Repeat) = [Sum_Torque;norm(Sum_Torque)];
            % Check_T_Common([1 2 3],Repeat) = Sum_Torque;
            % Check_T_Norm([1 2 3],Repeat) = Sum_Torque/norm(Sum_Torque);
        elseif Virtual_Switch == 1;
% Sum_Torque = Sum_Torque_att+Sum_Torque_rep;
    %          Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Virtual_Sum_Torque_att+Virtual_Sum_Torque_rep+ Sum_Zero_F_repTorque_rep_2;
             Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Virtual_Sum_Torque_att+Virtual_Sum_Torque_rep;
    %          Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2+Virtual_Sum_Torque_att+Virtual_Sum_Torque_rep;
    %          Sum_Torque = Sum_Torque_att+Sum_Torque_rep+Sum_Torque_rep_2+Virtual_Sum_Torque_att+Virtual_Sum_Torque_rep-Sum_Torque_att_Reverse;

        end

    %     Torque_Check = Sum_Torque'/norm(Sum_Torque)
        Pre_Theta = Theta;
        Theta = Theta + 0.005*(Sum_Torque'/norm(Sum_Torque)) ;
    %     Check_Theta = Theta * 180/pi
        % Check_Theta([1 2 3],Repeat) = Theta;


        % Att+rep �� (�ð�ȭ ���� ǥ���ϱ� ���� �������)
                Weight_F = 1;
                if Virtual_Switch == 0
                    for i = 1 : DOF
                       for j = 1 : length(mid_point)
                           F_Sum{i,j} = (F_att{i,j}+F_rep{i,j})*Weight_F;
                           F_Vatt{i,j} = F_att{i,j}*Weight_F;
                           F_Vrep{i,j} = F_rep{i,j}*Weight_F;
                       end
                    end
                elseif Virtual_Switch == 1;
                    if Virtual_Att_Progress_Timer >= 50
                        for i = 1 : DOF
                           for j = 1 : length(mid_point)
                               F_Sum{i,j} = (F_att{i,j}+F_rep{i,j}+Virtual_F_att{i,j}+Virtual_F_rep{i,j})*Weight_F;
                               F_Vatt{i,j} = (F_att{i,j}+Virtual_F_att{i,j})*Weight_F;
                               F_Vrep{i,j} = (F_rep{i,j}+Virtual_F_rep{i,j})*Weight_F;
                           end
                        end       
                    else
                         for i = 1 : DOF
                           for j = 1 : length(mid_point)
                               F_Sum{i,j} = (F_att{i,j}+F_rep{i,j}+Virtual_F_att{i,j})*Weight_F;
                               F_Vatt{i,j} = (F_att{i,j}+Virtual_F_att{i,j})*Weight_F;
                               F_Vrep{i,j} = (F_rep{i,j})*Weight_F;
                           end
                         end    
                    end
                end
    end   %������ �ٷ� ���� end��

    %% �׷��� �κ�
%   So_Plot_Origin(X,Y,Z, Goal_X,Goal_Y,Goal_Z, F_Sum, Repeat, Eraser ) % �̰� ������ �׷��� ��ֹ��� ������ ô������ ���� ��鸲

  So_Move_Plot(X,Y,Z, Goal_X,Goal_Y,Goal_Z, Ave_X,Ave_Y,Ave_Z , F_Sum, Repeat, Eraser,Judge_End )   % ô�¿� ���� ��帲�� ��հ����� ��������

    %%  �����ȣ�� �߸� ������
        if Judge_End ==1
            Final_Repeat = Repeat
            Record_Repeat(Change_Jaco_Potential+1) = Repeat;
            History_Theta(:,Repeat) = Goal_t*(180/pi); %�߰��� ������ �����Ƿ� ������ ���� �����Է�
            History_Ave_Theta(:,Repeat) = Goal_t*(180/pi); %�߰��� ������ �����Ƿ� ������ ���� �����Է�

            break;
        end

end






