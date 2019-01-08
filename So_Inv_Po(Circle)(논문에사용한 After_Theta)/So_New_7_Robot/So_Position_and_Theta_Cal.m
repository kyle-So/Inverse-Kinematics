function [ Sum_T ,R ] = So_Position_and_Theta_Cal( Theta1 ,Theta2 ,Theta3 ,Theta4 ,Theta5 ,Theta6, Theta7 , d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,alpha7 )
%SO_JACOBIAN_CAL Summary of this function goes here
%   Detailed explanation goes here

t1 = Theta1;
t2 = Theta2;
t3 = Theta3;
t4 = Theta4;
t5 = Theta5;
t6 = Theta6;
t7 = Theta7;

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
   
   
Sum_T = A1*A2*A3*A4*A5*A6*A7;


 NOA([1 2 3,4],[1 2 3,4])=   Sum_T;

Ini_Delta_X_N = atan2( -NOA(2,3) , NOA(3,3) );    %% ¿ä(x)
Ini_Delta_Y_O = atan2(  NOA(1,3) , NOA(3,3)*cos(Ini_Delta_X_N)-NOA(2,3)*sin(Ini_Delta_X_N) );
Ini_Delta_Z_A = atan2(  NOA(2,1)*cos(Ini_Delta_X_N)+NOA(3,1)*sin(Ini_Delta_X_N), NOA(2,2)*cos(Ini_Delta_X_N)+NOA(3,2)*sin(Ini_Delta_X_N));

Rotate_X = So_Rad_to_Deg( Ini_Delta_X_N);
Rotate_Y = So_Rad_to_Deg( Ini_Delta_Y_O);
Rotate_Z = So_Rad_to_Deg( Ini_Delta_Z_A);

R = roundn([Rotate_X;Rotate_Y;Rotate_Z],-4);

end

