The_X = 116
The_Y = 68
The_Z = -62

syms x y z
x = So_Theta_to_Rad(The_X)
y = So_Theta_to_Rad(The_Y)
z = So_Theta_to_Rad(The_Z)


Rx = [1   0       0   ;
      0 cos(x) -sin(x);
      0 sin(x)  cos(x)]

Ry = [cos(y) 0 sin(y);
       0     1    0;
     -sin(y) 0 cos(y)]
  
Rz =[cos(z) -sin(z) 0;
     sin(z)  cos(z) 0;
       0       0    1]
   
%   NOA = Rx*Ry*Rz
   NOA =    [-0.0007   -0.0001    1.0000    1.0171;
    0.0003    1.0000    0.0001    1.1946;
   -1.0000    0.0003   -0.0007    1.2902;
         0         0         0    1.0000]
%% Àý´ë ÁÂÇ¥

      

Ini_Delta_X_N = atan2( -NOA(2,3) , NOA(3,3) );    %% ¿ä(x)
Ini_Delta_Y_O = atan2(  NOA(1,3) , NOA(3,3)*cos(Ini_Delta_X_N)-NOA(2,3)*sin(Ini_Delta_X_N) );
Ini_Delta_Z_A = atan2(  NOA(2,1)*cos(Ini_Delta_X_N)+NOA(3,1)*sin(Ini_Delta_X_N), NOA(2,2)*cos(Ini_Delta_X_N)+NOA(3,2)*sin(Ini_Delta_X_N));

Rotate_X = So_Rad_to_Deg( Ini_Delta_X_N);
Rotate_Y = So_Rad_to_Deg( Ini_Delta_Y_O);
Rotate_Z = So_Rad_to_Deg( Ini_Delta_Z_A);

R = roundn([Rotate_X;Rotate_Y;Rotate_Z],-4)
   
   
   
%% »ó´ë ÁÂÇ¥
%  NOA([1 2 3,4],[1 2 3,4])=   DIDKD
% 
% Ini_Delta_Z_A = atan2(  NOA(2,1) , NOA(1,1) );
% Ini_Delta_Y_O = atan2( -NOA(3,1) , NOA(1,1)*cos(Ini_Delta_Z_A)+NOA(2,1)*sin(Ini_Delta_Z_A) );
% Ini_Delta_X_N = atan2( -NOA(2,3)*cos(Ini_Delta_Z_A)+NOA(1,3)*sin(Ini_Delta_Z_A), NOA(2,2)*cos(Ini_Delta_Z_A)-NOA(1,2)*sin(Ini_Delta_Z_A));
% 
% Rotate_X = So_Rad_to_Deg( Ini_Delta_X_N)
% Rotate_Y = So_Rad_to_Deg( Ini_Delta_Y_O)
% Rotate_Z = So_Rad_to_Deg( Ini_Delta_Z_A)