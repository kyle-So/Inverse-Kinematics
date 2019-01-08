function [ Position,X,Y,Z ] = So_Theta_to_Position( Theta )

global d a alpha mid_point

    t= Theta;

Numbe_of_Link = length(t) ;        %전체의 링크 수, 2축예니까 2라 적음

for i_Link = 1 : Numbe_of_Link
%     disp('전체계산 시작함')

% % % %     t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0];
% % % %     d = [1 0 3 0 3 0 1];
% % % %     a = [0 0 0 0 0 0 0];
% % % %     alpha = [90 -90 90 -90 90 -90 0];
    DH_Num=i_Link;       % DH 링크의 수 임시 이름...
          
% 행렬 A1,A2,A3 .... DH_Num 까지 행렬을 만듬
    for i=1:DH_Num

        A(:,:,i) =  [cos(t(i)) -sin(t(i))*cosd(alpha(i)) sin(t(i))*sind(alpha(i)) a(i)*cos(t(i));
                     sin(t(i)) cos(t(i))*cosd(alpha(i))  -cos(t(i))*sind(alpha(i)) a(i)*sin(t(i));
                     0            sind(alpha(i))            cosd(alpha(i))         d(i);
                     0                0                 0                1];

    end
    
    A1 = A(:,:,1)   ;   %이건 필요없는데 그냥 정의 예전에 A1는 현재의 이거다...정도?

% 이부분은 A1, A12, A123 계산하기.... 각 링크마다 자코비안 필요해서 뭐 이 자체로도 여러모로 쓰임
    for i = 1:DH_Num
    Help_Times=eye(4);       %행렬 곱하기 할때 도움주는 계수
        for j=1:i
             Help_Times = Help_Times*A(:,:,j);  % A1*A2 .... 계산하기
        end
            Times_A(:,:,i) = Help_Times ; % i번째 까지 곱한 행렬 A 값 
    end                                   %즉 A1, A12, A123 ..... 만들기
    
      
    for Num_mid = 1: length(mid_point)
       
       if i_Link == 1
            Position{i_Link,Num_mid} =  [Times_A([1 2 3],[4],i)*mid_point(Num_mid)];
            X{i_Link,Num_mid} =  [Times_A([1],[4],i)*mid_point(Num_mid)];
            Y{i_Link,Num_mid} =  [Times_A([2],[4],i)*mid_point(Num_mid)];
            Z{i_Link,Num_mid} =  [Times_A([3],[4],i)*mid_point(Num_mid)];
        else
            Position{i_Link,Num_mid} = [( ( Times_A([1 2 3],[4],i)-Times_A([1 2 3],[4],i-1) )*mid_point(Num_mid) ) + Times_A([1 2 3],[4],i-1)];
            X{i_Link,Num_mid} = [( ( Times_A([1],[4],i)-Times_A([1],[4],i-1) )*mid_point(Num_mid) ) + Times_A([1],[4],i-1)];
            Y{i_Link,Num_mid} = [( ( Times_A([2],[4],i)-Times_A([2],[4],i-1) )*mid_point(Num_mid) ) + Times_A([2],[4],i-1)];
            Z{i_Link,Num_mid} = [( ( Times_A([3],[4],i)-Times_A([3],[4],i-1) )*mid_point(Num_mid) ) + Times_A([3],[4],i-1)];
        end
        
    end
    
end
% X=[]; Y = [];
% 
% for i = 1: Numbe_of_Link
%     for j = 1 : length(mid_point)
%         X=[X Position{i,j}(1)];
%         Y=[Y Position{i,j}(2)];
%     end    
% end


end

