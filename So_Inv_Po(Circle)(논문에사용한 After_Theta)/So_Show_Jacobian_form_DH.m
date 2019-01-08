function [ So_Jacobian7 ] = So_Show_Jacobian_form_DH(t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7             )
%SO_SHOW_JACOBIAN_FOR_SHOW Summary of this function goes here
%   Detailed explanation goes here


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
       0                0                 0                1]  ;
   
A5 =  [cos(t5) -sin(t5)*cosd(alpha5) sin(t5)*sind(alpha5) a5*cos(t5);
       sin(t5) cos(t5)*cosd(alpha5)  -cos(t5)*sind(alpha5) a5*sin(t5);
       0            sind(alpha5)            cosd(alpha5)         d5;
       0                0                 0                1]   ;

A6 =  [cos(t6) -sin(t6)*cosd(alpha6) sin(t6)*sind(alpha6) a6*cos(t6);
       sin(t6) cos(t6)*cosd(alpha6)  -cos(t6)*sind(alpha6) a6*sin(t6);
       0            sind(alpha6)            cosd(alpha6)         d6;
       0                0                 0                1];
   
A7 =  [cos(t7) -sin(t7)*cosd(alpha7) sin(t7)*sind(alpha7) a7*cos(t7);
       sin(t7) cos(t7)*cosd(alpha7)  -cos(t7)*sind(alpha7) a7*sin(t7);
       0            sind(alpha7)            cosd(alpha7)         d7;
       0                0                 0                1];  
   
%% 자코비안 출력하기
Sum_T=(A1*A2*A3*A4*A5*A6*A7);

% 상수 일때 이거  

Position_7_Robot = Sum_T([1 2 3],4);

%% 심볼릭일때 요거   

% Position_7_Robot = simplify(Sum_T([1 2 3],4))
% Jaco = simplify(jacobian(Position_7_Robot,[t1,t2,t3,t4,t5,t6,t7]))

%% 손으로 자코비안   

% 회전 자코비안

R1 = A1([1 2 3],[1 2 3]);
R2 = A2([1 2 3],[1 2 3]);
R3 = A3([1 2 3],[1 2 3]);
R4 = A4([1 2 3],[1 2 3]);
R5 = A5([1 2 3],[1 2 3]);
R6 = A6([1 2 3],[1 2 3]);
R7 = A7([1 2 3],[1 2 3]);

z0 = [0;0;1];
z1 = R1 * z0;
z2 = R1*R2 * z0;
z3 = R1*R2*R3 * z0;
z4 = R1*R2*R3*R4 * z0;
z5 = R1*R2*R3*R4*R5 * z0;
z6 = R1*R2*R3*R4*R5*R6 * z0;

; %자코비안 아랫열 [w, ]

% 거리 구하기 위한 행렬연산 7축꺼를 써서 P7이라고 적혀있음
Distance_P7_P0 = Sum_T ;
Distance_P7_P1 = Sum_T - A1;
Distance_P7_P2 = Sum_T - A1*A2;
Distance_P7_P3 = Sum_T - A1*A2*A3;
Distance_P7_P4 = Sum_T - A1*A2*A3*A4;
Distance_P7_P5 = Sum_T - A1*A2*A3*A4*A5;
Distance_P7_P6 = Sum_T - A1*A2*A3*A4*A5*A6;

% 위에 거리 구하기 연산에서 4열행만 가져오기, 즉 행렬중 X,Y,Z값만 가져옴
P7_0 = Distance_P7_P0([1 2 3],[4]);  %변수명의 뜻 행렬 A1*A2*A3*A4*A5*A6*A7의 좌표값 - 행렬 A0의 좌표값
P7_1 = Distance_P7_P1([1 2 3],[4]);  %이건 A1*A2*A3*A4*A5*A6*A7 - A1  의 위치좌표값
P7_2 = Distance_P7_P2([1 2 3],[4]);
P7_3 = Distance_P7_P3([1 2 3],[4]);
P7_4 = Distance_P7_P4([1 2 3],[4]);
P7_5 = Distance_P7_P5([1 2 3],[4]);
P7_6 = Distance_P7_P6([1 2 3],[4]);

% 자코비안 각각 구한거
J1 =[cross(z0,P7_0);z0];
J2 =[cross(z1,P7_1);z1];
J3 =[cross(z2,P7_2);z2];
J4 =[cross(z3,P7_3);z3];
J5 =[cross(z4,P7_4);z4];
J6 =[cross(z5,P7_5);z5];
J7 =[cross(z6,P7_6);z6];

% 7축 자코비안 합친거
% disp('-------------------------------------------------- ');
% disp('로봇 6축 자코비안');
So_Jacobian7 = [J1 J2 J3 J4 J5 J6,J7];

% So_Jacobian5  ;           %상수일때 자코비안 행렬
% Jacos = simplify(So_Jacobian5)   %심볼릭일 떄 자코비안 행렬

% disp('-------------------------------------------------- ');

end

