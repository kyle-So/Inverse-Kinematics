clc
j= 0;
for i = 1 : (Repeat-1)/14 : Repeat
    j=j+1
    i_num=round(i)
    His_Theta_10(:,j) = History_Ave_Theta(:,i_num);
    
end

Result_Theta = His_Theta_10'