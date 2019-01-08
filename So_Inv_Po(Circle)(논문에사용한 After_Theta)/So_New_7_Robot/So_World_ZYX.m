function [ R ] = So_World_ZYX( t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 )
%SO_WORLD_ZYX Summary of this function goes here
%   Detailed explanation goes here

% Rx = [1   0       0   ;
%       0 cos(x) -sin(x);
%       0 sin(x)  cos(x)];
% 
% Ry = [cos(y) 0 sin(y);
%        0     1    0;
%      -sin(y) 0 cos(y)];
%   
% Rz =[cos(z) -sin(z) 0;
%      sin(z)  cos(z) 0;
%        0       0    1];
%    
%    World_RxRyRz = Rx*Ry*Rz;
   
Sum_T=So_Show_Matrix_from_DH(  t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7, a1,a2,a3,a4,a5,a6,a7, alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7 );

   NOA=   Sum_T;

Ini_Delta_X_N = atan2( -NOA(2,3) , NOA(3,3) );    %% ¿ä(x)
Ini_Delta_Y_O = atan2(  NOA(1,3) , NOA(3,3)*cos(Ini_Delta_X_N)-NOA(2,3)*sin(Ini_Delta_X_N) );
Ini_Delta_Z_A = atan2(  NOA(2,1)*cos(Ini_Delta_X_N)+NOA(3,1)*sin(Ini_Delta_X_N), NOA(2,2)*cos(Ini_Delta_X_N)+NOA(3,2)*sin(Ini_Delta_X_N));

Rotate_X = So_Rad_to_Deg( Ini_Delta_X_N);
Rotate_Y = So_Rad_to_Deg( Ini_Delta_Y_O);
Rotate_Z = So_Rad_to_Deg( Ini_Delta_Z_A);

R = roundn([Rotate_X;Rotate_Y;Rotate_Z],-4);

end

