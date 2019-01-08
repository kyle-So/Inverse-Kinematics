clc
clear

%% 나중에는 다 받을파일. 지금은 함수파일이 아니므로 적어둠
global d a alpha mid_point
Ini_t = [(pi/180)*15 (pi/180)*30];
Goal_t = [(pi/180)*90 (pi/180)*0];     
    d = [0 0];
    a = [3 2];
    alpha = [0 0];
    
Coeff_att = 1;
Coeff_rep = 1;
mid_point = [0.5 1];
DOF = length(Ini_t);
Object = [3;3;0];
Theta = Ini_t;
Bound_con = 1;
L_end = length(mid_point);
%% 실제는 여기부터

[ P_All,X,Y,Z]=So_Theta_to_Position( Theta )

figure(1)
hold on

xlabel('x')
ylabel('y')
zlabel('z')

for i = 1 : DOF
       if i == 1
           disp('반복')
           plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2) 
       else
           disp('반복')
           plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) 
       end
end

      
for i = 1 : DOF
   for j = 1 : length(mid_point)
      plot3(X{i,j},Y{i,j},Z{i,j},'.', 'Color', 'r', 'LineWidth', 3)
   end
end

