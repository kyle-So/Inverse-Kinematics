clc

p0 = 5
N = sqrt( 11.25   )
V = [-1.5;-3]

E1 = ( 1/N - 1/p0)
E2 = 1/(N^3)

So_ANS = E1*E2*V

% D_Norm{1,1} = N;
% D_Vec{1,1} = V;
% Bound_con = 
% 
% Check=( (1/D_Norm{i,j}) - (1/Bound_con) ).*(1/D_Norm{i,j})^3 * (D_Vec{i,j});