function [ Position,X,Y,Z ] = So_Theta_to_Position( Theta )

global d a alpha mid_point

    t= Theta;

Numbe_of_Link = length(t) ;        %��ü�� ��ũ ��, 2�࿹�ϱ� 2�� ����

for i_Link = 1 : Numbe_of_Link
%     disp('��ü��� ������')

% % % %     t = [(pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0 (pi/180)*0];
% % % %     d = [1 0 3 0 3 0 1];
% % % %     a = [0 0 0 0 0 0 0];
% % % %     alpha = [90 -90 90 -90 90 -90 0];
    DH_Num=i_Link;       % DH ��ũ�� �� �ӽ� �̸�...
          
% ��� A1,A2,A3 .... DH_Num ���� ����� ����
    for i=1:DH_Num

        A(:,:,i) =  [cos(t(i)) -sin(t(i))*cosd(alpha(i)) sin(t(i))*sind(alpha(i)) a(i)*cos(t(i));
                     sin(t(i)) cos(t(i))*cosd(alpha(i))  -cos(t(i))*sind(alpha(i)) a(i)*sin(t(i));
                     0            sind(alpha(i))            cosd(alpha(i))         d(i);
                     0                0                 0                1];

    end
    
    A1 = A(:,:,1)   ;   %�̰� �ʿ���µ� �׳� ���� ������ A1�� ������ �̰Ŵ�...����?

% �̺κ��� A1, A12, A123 ����ϱ�.... �� ��ũ���� ���ں�� �ʿ��ؼ� �� �� ��ü�ε� ������� ����
    for i = 1:DH_Num
    Help_Times=eye(4);       %��� ���ϱ� �Ҷ� �����ִ� ���
        for j=1:i
             Help_Times = Help_Times*A(:,:,j);  % A1*A2 .... ����ϱ�
        end
            Times_A(:,:,i) = Help_Times ; % i��° ���� ���� ��� A �� 
    end                                   %�� A1, A12, A123 ..... �����
    
      
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

