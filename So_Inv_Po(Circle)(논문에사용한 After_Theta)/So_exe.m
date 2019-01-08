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

% % UR 초기값 및 결과
% %  Ini_t = [(pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90  (pi/180)*90];
%  Ini_t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0  (pi/180)*0];
% Goal_t = [(pi/180)*(33) (pi/180)*33 (pi/180)*(-41) (pi/180)*76 (pi/180)*(-9) (pi/180)*(-15)  (pi/180)*26];     
% % Goal_t = [(pi/180)*(-3.6313) (pi/180)*29.3335 (pi/180)*(16.9509) (pi/180)*75.8842 (pi/180)*(45.5186) (pi/180)*(-28.4714)  (pi/180)*15];
% 
%     d = [278 0 425 0 425 0 72];
%     a = [0 0 0 0 0 0 0];
%     alpha = [-90 90 -90 90 -90 90 0];
%  History_Ave_Theta(:,1) = Ini_t*(180/pi);  

% 학교 로봇 실험용
%  Ini_t = [(pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90 (pi/180)*90  (pi/180)*90];
%  Ini_t = [(pi/180)*(-73.263) (pi/180)*(68.5316) (pi/180)*(108.1991) (pi/180)*(109.287) (pi/180)*(214.9152) (pi/180)*(42.131)  (pi/180)*(81.4465)];
% Goal_t = [(pi/180)*(-38.4599) (pi/180)*(86.8663) (pi/180)*(120.815) (pi/180)*(109.2882) (pi/180)*(200.4894) (pi/180)*(76.5099)  (pi/180)*(65.823)];     
% Goal_t = [(pi/180)*(-3.6313) (pi/180)*29.3335 (pi/180)*(16.9509) (pi/180)*75.8842 (pi/180)*(45.5186) (pi/180)*(-28.4714)  (pi/180)*15];

% [ Ini_t ] = So_Fine_Theta_Cal(  550, -200 ,650 , 0 , 45 , -45 )   % 초기위치
% Ini_t = [-1.0699    1.5140    2.3533    1.3448    2.8469    0.1707    0.7697] ;  % 실험에쓴거
% [ Goal_t ] = So_Fine_Theta_Cal( 550, 300 ,650 , 0 , 45 , -45 )  % 목표위치
% Goal_t = [0.1419    1.3750    2.1392    1.1397    2.4698    0.9998    0.4809] ;  % 실험에쓴거

% 원 3차 실험값
% Ini_t = [-0.2493    0.4194   -0.4166    1.3447   -0.3343   -1.1174   -0.1171];
% Goal_t = [0.6641    0.5357    0.0191    1.1398    0.5201   -1.0789   -1.5466];

%이건 그냥 해보는거
Ini_t = [-0.1696    0.3759   -0.5518    1.3056   -1.2812   -0.7048    1.4713];
Goal_t = [1.3375    0.8273   -1.3972    1.0976   -0.0953    0.2908    0.1438];

    d = [278 0 425 0 425 0 72];
    a = [0 0 0 0 0 0 0];
    alpha = [-90 90 -90 90 -90 90 0];
 History_Ave_Theta(:,1) = Ini_t*(180/pi);  
Obs_Circle{1} = [550 150 580];
% Obs_Circle{2} = [-200 500 0];
Obs_Cir_Radius = [100 0];

Change_Jaco_Potential =  0 ;  %몇번 변화했는지 확인 스위치 초기값 0
SW_Jacobian = 0;            % 자코비안쪽 스위치
SW_Potential = 0;           % 포텐셜쪽 스위치
%% DH 및 초기값 정의
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
           Virtual_F_att{i,j}= [0;0;0] ;       %에러방지 초기값 설정
           Virtual_F_rep{i,j}= [0;0;0] ;
     end
end 

for i = 1: 10
    Ave_10_Theta(i,:) = Ini_t*0 ;
end

So_Coeff;  %가중치 불러오기

Eraser = 1;
Theta = Ini_t;
Before_Check = zeros(DOF,L_end);
% 2D일때 장애물
% Obstacle{1} = [3 3 0 ; 3 4 0; 4 4 0; 4 3 0; 3 3 0];       % 1번 장애물

% Obs_Circle{1} = [300 400 0];
% Obs_Circle{2} = [-200 500 0];
% Obs_Cir_Radius = [0 100];

% 7축일때 장애물
% Obs_Circle{1} = [400 0 800];
% Obs_Circle{2} = [6.5 2 0];
% Obs_Cir_Radius = [100];

obs_thrs = [5 5 5 5]  ;                         % 장애물 경계값
% Transe_Obs= Obstacle{1}' ;                       % 세로로 좌표 표현
% Bound_con = [0.3 0.1]  
Bound_con = Obs_Cir_Radius*0.2        % 경계값을 반지름으로 잡음
Bound_con_2 = Obs_Cir_Radius*1        %반지름 * 0.5 = 반지름의 반이 경계값
 
Try_Num = 2000;
Limit_Value =1;
Process_Potential = 0 ;

 [Ave_P_All,Ave_X,Ave_Y,Ave_Z]=So_Theta_to_Position( Ini_t ) ;  
%% 장애물

% 2D 사각형
% 	for i = 1:length(Obstacle)
% 		obs = Obstacle{i};
% 		patch(Obstacle{i}(:,1),Obstacle{i}(:,2),Obstacle{i}(:,3),'black');  % x,y 에 색 채움
%     end
    
% 3D 구
figure(1)
Display_Coef = 1;         % 반지름이 얼마의 사이즈로 보일것인가? 1이면 실제 반지름 
hold on
for i = 1 : length(Obs_Circle)
[Cir_x,Cir_y,Cir_z] = sphere;               %일단 구 생성
surf((Obs_Cir_Radius(i)*Display_Coef*Cir_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*Display_Coef*Cir_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*Display_Coef*Cir_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)    % 크기 3짜리 5,4,0에 생성
colormap winter
grid off
end
% view(45,20)
view(114,32)
% view(84,16)
% % 3D 실린더
% figure(1)
% Display_Cylinder_Coef = 1;         % 반지름이 얼마의 사이즈로 보일것인가? 1이면 실제 반지름 
% hold on
% for i = 1 : length(Obs_Circle)
% [Cyl_x,Cyl_y,Cyl_z] = cylinder;               %일단 구 생성
% surf((Obs_Cir_Radius(i)*Display_Cylinder_Coef*Cyl_x)+Obs_Circle{i}(1),(Obs_Cir_Radius(i)*Display_Cylinder_Coef*Cyl_y)+Obs_Circle{i}(2),(Obs_Cir_Radius(i)*10*Display_Cylinder_Coef*Cyl_z)+Obs_Circle{i}(3),'FaceAlpha',0.4,'EdgeAlpha',0.1)    % 크기 3짜리 5,4,0에 생성
% colormap winter
% grid off
% end



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
    [ Ini_All,Ini_X,Ini_Y,Ini_Z]=So_Theta_to_Position( Ini_t );
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
    
    [ Switch_Obs,Norm_Q_to_Obs, Switch_F_Rep,Judge_Matrix_0, Judge_Matrix ] = So_Judge_Obstacle( Goal_All, P_All );       %현재위치와 마지막 위치 가는길에 장애물이 있는가 없는가?
%     Switch_Obs
    % 장애물이 없으면 굳이 포텐셜 필드 사용할 필요 X
    % 따라서 현재 각도와 목표 각도를 일정하게 나눠봄
    
%     JM_0 = Judge_Matrix_0(:,:,1)
%     JM = Judge_Matrix(:,:,1)
%     Repeat;
    if and(max(Switch_Obs) == 0,Process_Potential == 0)   % 가는길에 장애물이 없다면
        SW_Jacobian = SW_Jacobian + 1;
            if SW_Jacobian == 1
                Change_Jaco_Potential = Change_Jaco_Potential + 1;  % 변화량 1 증가
                SW_Potential = 0;   % 포텐셜스위치는 Off
                Record_Repeat(Change_Jaco_Potential) = Repeat;
            end
            
        [ Divide_Number,Judge_End ] = So_Distance_P_G( Ini_All,P_All,Goal_All );  % 거리에 따른 등분
        
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
                Change_Jaco_Potential = Change_Jaco_Potential + 1;  % 변화량 1 증가
                SW_Jacobian = 0;   % 포텐셜스위치는 Off
                Record_Repeat(Change_Jaco_Potential) = Repeat;
            end              
        
    % else문... 여기서부터는 포텐셜 필드
    Process_Potential = 1 ;   % 포텐셜 필드 시작했다는 뜻. 이러면 역기구학 접근이 불가능함 포텐셜이 필요없다고 판단되면 이 과정이 0이됨.
                              % 언제 0이 되야 할 것인가?
        % 진짜 계산 시작.

        % Att벡터와 장애물 사이의 거리
%         [ Norm_Q_to_Obs, Switch_F_Rep ] = So_Distance_Att_Obs( Goal_All, P_All )  % 상위 호환 파일인 So_Judge_Obstacle에 이 기능 넣음
%         Minimum_Vatt_Obs = min(min( cell2mat(Norm_Q_to_Obs) ))   ;              %로봇암과 목표지점까지의 직선벡터 Att와 장애물 Obs의 최소거리
    %     Switch_F_Rep

        % 
% Switch_F_Rep

        % Att 힘
        [ F_att,Unit_Vec_att  ] = So_F_att( Goal_All,P_All );

    %     FATT = cell2mat(F_att)

    % Rep 힘
    % 0차 : 바운더리 거의 없음(0.001?) 그냥 순수한 물체의 반지름 -> 기본 포텐셜 척력사용
    % 1차 : 바운더리 아주 얇음 0.3r? 역시 기본 포텐셜사용
    % 2차 : 0,1차 척력발생시 강력한 척력으로 인해 급격한 속도 가속도 변화가 일어남 -> 기구무리
    % 그렇기에 선형적인 감속, 가속이 일어나도록 수정된 척력 방식 사용
    % 어쨋든, 척력 추가할때마다 밑에 항에서 척력 식 추가하기 귀찮으므로 여기서 척력을 하나로 합침
    
      % 0차 Rep 힘 (바운더리없이 바로 장애물)
        [ Origin_Zero_F_rep ] = So_Zero_F_rep( Goal_All,P_All );
            for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle) %일단 0
                     F_rep_0{i,j,Num_Obs}=Origin_Zero_F_rep{i,j,Num_Obs};  % 0차는 스위치가 켜지던 꺼지던 항상 척력이 있음  
                   end
               end
            end   % 변수에 따른 충돌 방지 각 링크 각 질점마다의 힘
  
        % 1차 Rep 힘
        [ Origin_F_rep ] = So_F_rep( Goal_All,P_All );
            for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle)
                     F_rep_1{i,j,Num_Obs}=Origin_F_rep{i,j,Num_Obs}.*Switch_F_Rep(Num_Obs);  
                   end
               end
            end     % F_rep 각 링크 각 질점마다의 힘

        % 2차 방어선 Rep 힘  ATT 이용해만든값
        [ Origin_F_rep_2 ]=So_Boundary2_Rep( Goal_All, P_All, F_att, Unit_Vec_att  );    
             for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle)
                     F_rep_2{i,j,Num_Obs}=Origin_F_rep_2{i,j,Num_Obs}.*Switch_F_Rep(Num_Obs);  % 0000000  
                   end
               end
             end   % 2차 방어선 F_rep 각 링크 각 질점마다의 힘 
            
        %척력 Rep, 0,1,2차 합치기
            for i = 1 : DOF
               for j = 1 : length(mid_point)
                   for Num_Obs = 1 : length(Obs_Circle)
                     F_rep{i,j,Num_Obs}=F_rep_0{i,j,Num_Obs}+F_rep_1{i,j,Num_Obs}+F_rep_2{i,j,Num_Obs}; % F_rep는 012다 합친거
                   end
               end
            end   


    % F2= cell2mat(F_rep_2)
        %확인용

    %     [cell2mat(Origin_F_rep) cell2mat(F_rep)];

        % 장애물이 없으면 인력,척력의 계수는 동등하게.
        if max(Switch_F_Rep) == 0         %장애물이 없으면 스위치 킬 준비  
            Coeff_Switch = Coeff_Switch +1;
        else
             Coeff_Switch = 0;
        end

        if Coeff_Switch > 20            %스위치가 20개 쌓이면 장애물이 충분히 없다고 판단.  
            So_Coeff_0               % 모든계수 1
            Process_Potential = 0  
        else
            So_Coeff
        end





        % i번째의 각 질점에서의 방향, i-1번째에서의 방향, 그리고 i,i-2방향의 방향(이거는 뒤로갔다 다시 앞으로 갔을 때 그 차이 볼려고 계산
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

                    [Current_Diff_XYZ Pre_Current_Diff_XYZ];  % 이거 곱해서 음수가 나오면 부호가 바뀌는거
            %         [Pre_Diff_Two_XYZ Diff_Two_XYZ] %결과적으로 이 두값은 거의 같다 빼기 조건으로 못씀
                    % 변화량의 부호가 변화고, -2의 차가 작은경우를 조건으로 쓰면?
             end

          if Repeat >= 4 
            % 로컬미니멈 위치 찾기
            if Current_Diff_XYZ.*Pre_Current_Diff_XYZ <= 0     % i, i-1번째의 방향이 반대이면
%                 Check_Switch_Timer = Check_Switch_Timer+5;     % 로컬미니멈 의심가니까 일단 카운터 올림
Check_Switch_Timer = 0; % 임시 값 로컬미니멈없앰
            else
                Check_Switch_Timer = 0;                        % 두 방향차이가 없다면 일단 로컬 미니멈 아니니까 카운터 초기화
            end

            if Check_Switch_Timer > 100
%                     if max(abs(Diff_Two_XYZ)) < 0.01      %두번째 조건, 이동거리가 입실론보다 작으면
                                                % 이 조건 스케일에대한 고려가 필요함
                       if max(abs(Diff_Two_XYZ)) < 1 % 일단 스케일이 500 600 넘어가므로 1로함                        
                        Local_Minimum_Timer = Local_Minimum_Timer+5     % 질점의 움직임이 이동거리보다 작고, 속도의 방향이 반대, 2조건을 만족하므로 로컬미니멈 카운터 

                        if and(Local_Minimum_Timer >= 80, Local_Minimum_Timer <= 100)   % 이 부분 숫자가 고민인데, 로컬미니멈을 밀어내기위한 가상의 반력 생성을 어디다 할것인가... 
                            Divid = Divid +1;
                            Min_Point = Min_Point + [Mat_X(DOF,L_end);Mat_Y(DOF,L_end);Mat_Z(DOF,L_end)];
                            Average_LM=Min_Point/Divid;         %어쨋든, Average x,y,z에 만드는데 조금 보강할 필요가있어보임.
                        end

                    end
            end
         end


        %여기까지 계산하고 Local_Minimum_Timer이 100이 넘으면 로컬미니멈에 빠졌다고 할 수 있다.

            % 로컬미니멈일때 각종 스위치 ON,OFF 기능
            if Local_Minimum_Timer == 100    %로컬미니멈은 계속 증가하지만 일단 100이 넘으면 스위치 올림
                Parameter_Switch = 1;        % 이 파라메터는 가상의 반력 파라메터를 On
            end


            %로컬 미니멈 100 넘을때 반력 생성
        if and(Parameter_Switch == 1,Cre_Plot==1)
            Vir_Obs_Cir_Radius = 100;
            [Virtual_RepX,Virtual_RepY,Virtual_RepZ] = sphere;               %일단 구 생성
            surf( Vir_Obs_Cir_Radius*(Virtual_RepX)+Average_LM(1) , Vir_Obs_Cir_Radius*(Virtual_RepY)+Average_LM(2), Vir_Obs_Cir_Radius*(Virtual_RepZ)+Average_LM(3),'FaceAlpha',0.2,'EdgeAlpha',0)      % Average x,y,z
        %     colormap winter
            grid off
            Cre_Plot = 0;  % 혹시 모를 반복에대비한 또 다른 안전장치
            Virtual_Switch = 1;       %로컬미니멈 100되는순간 Ex포텐셜 스위치 ON
            Virtual_Obs_Circle{Virtual_Num_Obs} = [Average_LM(1) Average_LM(2) Average_LM(3)] ;
            Virtual_Num_Obs = Virtual_Num_Obs+1;  % 다음 가상의 힘 대비해서 Index 번호 올림
            Virtual_Bound_con = 1;           % 가상의 구 경계값
            Virtual_Att_Progress_Timer=0;    % 추가반력 인력을 동시 생성하면 반력벡터문제가 발생할 수 있으므로 인력을 먼저 대입하기위한 타이머 초기화
        end

      % 가상 Att 힘
        if Virtual_Switch == 1
    %         Local_Minimum_Timer = 0;   % Ex포텐셜 스위치 ON되는순간 로컬미니멈 0 (다음 로컬미니멈 계산을 위한 초기화)
          [ Virtual_Sum_Torque_att,Virtual_F_att  ] = So_Virtual_Att( P_All,J );
            Virtual_Att_Progress_Timer = Virtual_Att_Progress_Timer + 1;      % 가상 척력 만들기전 사전 대기시간
        end

        % 타이머
        if Virtual_Att_Progress_Timer >= 50
            Virtual_Rep_Switch =1;          % 가상의 인력이 50번 작용했을때 가상척력도 생성
        end

        %가상 척력 Rep 힘
        if Virtual_Rep_Switch == 1
           [ Virtual_Sum_Torque_rep, Virtual_F_rep ] = So_Virtual_Rep( P_All,J,Virtual_Obs_Circle );
        end

        % 마지막 링크의 가상 인력 Vri_Att은 장애물이 없으면 항상 가상인력 없애면???

        
%                    for Num_Obs = 1 : length(Obs_Circle)
                        if(Judge_Matrix(DOF,L_end,1)) ==0
                            Virtual_F_att{DOF,L_end} = [0;0;0];
                            Virtual_F_att{DOF,1} = [0;0;0];
                            Virtual_Sum_Torque_att(3,1) = 0;
                            Sum_Torque_att(3,1) = Sum_Torque_att(3,1) * 5;  %기존 가상인력이 10배 주니까 해제하는대신 5배 추가... 근데 이거 해제해야하나..?
                        end
%                    end

        
        
        
            % 척력 해제 조건. 점과 직선의 거리를 만들었으니까 필요없음 좀 수정 해야함..
        if Virtual_Switch == 1

            if Virtual_Att_Progress_Timer >= 11
                if max(Switch_F_Rep) == 0  
    %                 B_Alpha = 0.5;
    %                 Bound_con = Obs_Cir_Radius*B_Alpha    ; % 임시로 사용 나중에는 바운더리 컨디션을 위에서 바꾸도록 설계
                   for i = 1 : DOF
                         for j = 1 : length(mid_point)
                               Virtual_F_att{i,j}= [0;0;0] ;       % 2축이라 Z는 제거함
                                              % Virtual_Att 힘-> 토크
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

            % 힘-> 토크
        for i = 1 : DOF
           for j = 1 : length(mid_point)
                   % Att 힘-> 토크
               Torque_att{i,j} = J{i,j}'*(F_att{i,j});
               Sum_Torque_att = Sum_Torque_att+Torque_att{i,j};
    %            Torque_att_Reverse{i,j} = J{i,j}'*(F_att_Reverse{i,j})*Limit_Value;
    %            Sum_Torque_att_Reverse = Sum_Torque_att_Reverse+Torque_att_Reverse{i,j};

               for Num_Obs = 1 : length(Obs_Circle)
                   % Rep 힘-> 토크
                   Torque_rep{i,j,Num_Obs} = J{i,j}'*F_rep{i,j,Num_Obs}*Limit_Value;
                   Sum_Torque_rep = Sum_Torque_rep+Torque_rep{i,j,Num_Obs};
%                    Torque_rep_2{i,j,Num_Obs} = J{i,j}'*F_rep_2{i,j,Num_Obs}*Limit_Value;
%                    Sum_Torque_rep_2 = Sum_Torque_rep_2+Torque_rep_2{i,j,Num_Obs};      
                   %
                   %0차 방어선
%                    Torque_Zero_rep_2{i,j,Num_Obs} = J{i,j}'*Zero_F_rep{i,j,Num_Obs}*Limit_Value;
%                    Sum_Zero_F_repTorque_rep_2 = Sum_Zero_F_repTorque_rep_2+Torque_Zero_rep_2{i,j,Num_Obs};                

               end
           end
        end
%         cell2mat(Torque_att);
%         cell2mat(Torque_rep)
    % D_Norm
    % F_rep{2,2}

        % 비교를 위해 att만
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


        % Att+rep 힘 (시각화 벡터 표현하기 위해 만들었음)
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
    end   %각도로 바로 가기 end임

    %% 그래픽 부분
%   So_Plot_Origin(X,Y,Z, Goal_X,Goal_Y,Goal_Z, F_Sum, Repeat, Eraser ) % 이건 원래의 그래프 장애물이 있으면 척력으로 인해 흔들림

  So_Move_Plot(X,Y,Z, Goal_X,Goal_Y,Goal_Z, Ave_X,Ave_Y,Ave_Z , F_Sum, Repeat, Eraser,Judge_End )   % 척력에 의한 흔드림을 평균값으로 보상해줌

    %%  종료신호가 뜨면 끝내기
        if Judge_End ==1
            Final_Repeat = Repeat
            Record_Repeat(Change_Jaco_Potential+1) = Repeat;
            History_Theta(:,Repeat) = Goal_t*(180/pi); %중간에 강제로 끊으므로 마지막 값은 수동입력
            History_Ave_Theta(:,Repeat) = Goal_t*(180/pi); %중간에 강제로 끊으므로 마지막 값은 수동입력

            break;
        end

end






