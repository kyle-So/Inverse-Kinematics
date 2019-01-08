function [ output_args ] = So_SumT_for_Cal( Theta1,Theta2,Theta3,Theta4,Theta5,Theta6 )
%SO_SUMT_FOR_CAL Summary of this function goes here
%   Detailed explanation goes here
t1    ; d1=1          ; a1=0         ;  alpha1 =   90;
t2    ; d2=0          ; a2=0         ;  alpha2 =  -90;
t3    ; d3=3          ; a3=0         ;  alpha3 =   90;
t4    ; d4=0          ; a4=0         ;  alpha4 =  -90;
t5    ; d5=3          ; a5=0         ;  alpha5 =   90;
t6    ; d6=0          ; a6=0         ;  alpha6 =  -90;
t7    ; d7=1          ; a7=0         ;  alpha7 =    0;

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
   
A7 =  [cos(t7) -sin(t7)*cosd(alpha7) sin(t7)*sind(alpha7) a7*cos(t7);
       sin(t7) cos(t7)*cosd(alpha7)  -cos(t7)*sind(alpha7) a7*sin(t7);
       0            sind(alpha7)            cosd(alpha7)         d7;
       0                0                 0                1];  
   
disp('목표값 Forward 값')   
Sum_T=(A1*A2*A3*A4*A5*A6*A7)

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

disp('목표값 Forward 해석')
DP_XYZ = [Position_X;Position_Y;Position_Z;Rotate_X;Rotate_Y;Rotate_Z]


end

