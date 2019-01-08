%% �� ������ So_Jacobian�� 7�� �ڵ�� 
%  ���ټ��ʵ尡 X,Y,Z ���� �䱸�ϹǷ� ���������̼��� ���� ������ ����߱⿡ �������� ���������� ���ܵ�



function [ Use_Jaco ] = So_Jacobian( Theta )      

global d a alpha mid_point

    t= Theta;
%******************************************************
    
Number_of_Link = length(t);         %��ü�� ��ũ ��, 2�࿹�ϱ� 2�� ����
for i_Link = 1 : Number_of_Link
%     disp('��ü��� ������')

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
    
% �ʱⰪ ���� �κ� ȸ���κ�, �׸��� ���ں�� ����ϱ����� ������ x,y,z�κ� ��� �ʱⰪ
    z0 = [0;0;1];
    Z(:,:,1)=[0;0;1];
    Distance_A(:,:,1) = Times_A(:,:,DH_Num);      %�ʱⰪ ����
    Sum_J{i_Link}=[];
    Sum_Z{i_Link}=[];
% ������ ���ں�� ��� �κ�, �ӵ��� ��� ���� z,��ġ�κ��� �߰�,
% ���������̼��� ��� R1*..R(n-1)* [0;0;1] �ϸ� ��
% ���������� �����ϸ� �翬�� �Ҹ�. �ڼ��Ѱ� �ѱ� ���ں�� �������� ����(�˰� ���°����� Ȥ�� �ؾ��������..)
% �׷��� ��������� �̺κ��� ���ں���� ���ϴ°ǵ� mid_point�� ����ϹǷ� �ӵ�, ���ӵ� �κ��� ����
    for i = 1 : DH_Num
%         disp('���� ���ں�� �����')
         R(:,:,i) = Times_A([1 2 3],[1 2 3],i) ;  % ������� R2 �� R1*R2�� ����
         Z(:,:,i+1) = R(:,:,i)*[0;0;1]  ;         % ���ں�� ���������̼ǰ�
         Distance_A(:,:,i+1) = Times_A(:,:,DH_Num)-Times_A(:,:,i);
         Position_A(:,:,i) = Distance_A([1 2 3],[4],i);   % mid_point�� ȸ���� �������
         J(:,:,i) = [cross(Z(:,:,i),Position_A(:,:,i)) ; Z(:,:,i)];  % �̰� ���� �ӵ�,ȸ�� ���ں�� �ΰ� �� ������ ����
%                                                                      %������ mid_point�� �� ����� �ʿ��ϹǷ� �ӵ��κи� ���� ��� 
         Vel_J(:,:,i) = [cross(Z(:,:,i),Position_A(:,:,i))];    % ������(�ӵ�)�κи� ���
         
         Sum_J{i_Link} = [Sum_J{i_Link} Vel_J(:,:,i)] ;                                   
         Sum_Z{i_Link} = [Sum_Z{i_Link} Z(:,:,i)] ;
%          PREPREPREPRE =Sum_J{i_Link}

    end
    
             
         if DH_Num < Number_of_Link
%              disp('���ǹ�����')
             Sum_J{i_Link} = [Sum_J{i_Link} zeros(3,Number_of_Link-DH_Num)];
             Sum_Z{i_Link} = [Sum_Z{i_Link} zeros(3,Number_of_Link-DH_Num)];
%              RRRR=Sum_J{i_Link}
         else
%              disp('��������')
             Sum_J{i_Link};
             Sum_Z{i_Link};
%              EEEE=Sum_J{i_Link}
         end
    
    
    
%     disp('�˻����')
    Sum_J{:};   % �˻�� ���߿� �ּ�ó��
                 % �ᱹ ���ں�� ����ü ó���� ���� 1�� A1 ,2�� A12 ,3�� A123 ���� ���ں�� ���Ѱ�
                 % �Ʒ����� ���� ����ü�� �������� �������.
    

% disp('����ü �����κ�')
% i_Link = ������ ��ũ �� A1, A12, A123 ��ũ�� ���� C_Link=3 �̶�� A123�� �����ִ°���
      
        for Num_mid = 1: length(mid_point)
    
            if i_Link == 1
            Structure_Jaco{i_Link,Num_mid} =  [Sum_J{i_Link}*mid_point(Num_mid);Sum_Z{i_Link}] ;
            else
            Structure_Jaco{i_Link,Num_mid} = [( ( Sum_J{i_Link}-Sum_J{i_Link-1} )*mid_point(Num_mid) ) + Sum_J{i_Link-1};Sum_Z{i_Link}];    
            end
        end
         

end

% cellplot(Structure_Jaco)

% Structure_Jaco{7,2}

Remake_End = Number_of_Link; 

for i = 1: Number_of_Link
    for j = 1 : length(mid_point)
        Use_Jaco{i,j}=[];
        for Remake = 1:Number_of_Link 
            if Remake < 7
                Use_Jaco{i,j}=[Use_Jaco{i,j} ;  Structure_Jaco{i,j}(Remake,:)];
            end
        end
    end
       
end

% cellplot(Use_Jaco)

% disp('���')
Use_Jaco{:,:};






% Structure_Jaco{2,2}
% cellplot(Structure_Jaco)

% % 
% % 
% % %% ���� ���
% % 
% % t1 =180*0    ; d1=0          ; a1=3         ;  alpha1 =  0;
% % t2 =180*0    ; d2=0          ; a2=2         ;  alpha2 =  0;
% % DH_Num=2; 
% % mid_point = [0.5 1];
% % A1 =  [cos(t1) -sin(t1)*cosd(alpha1) sin(t1)*sind(alpha1) a1*cos(t1);
% %        sin(t1) cos(t1)*cosd(alpha1)  -cos(t1)*sind(alpha1) a1*sin(t1);
% %        0            sind(alpha1)            cosd(alpha1)         d1;
% %        0                0                 0                1];
% %    
% % A2 =  [cos(t2) -sin(t2)*cosd(alpha2) sin(t2)*sind(alpha2) a2*cos(t2);
% %        sin(t2) cos(t2)*cosd(alpha2)  -cos(t2)*sind(alpha2) a2*sin(t2);
% %        0            sind(alpha2)            cosd(alpha2)         d2;
% %        0                0                 0                1];
% % 
% %    
% % %% ���ں�� ����ϱ�
% % 
% % A12 = A1*A2
% % % ��� �϶� �̰�  
% % % Position_A1 = A1([1 2],4);
% % 
% % %% �ɺ����϶� ���   
% % 
% % % P_A1 = simplify(A1([1 2],4))
% % % Jaco_A1 = simplify(jacobian(P_A1,[t1,t2]))
% % % 
% % % P_A12 = simplify(A12([1 2],4))
% % % Jaco_A12 = simplify(jacobian(P_A12,[t1,t2]))
% % %% ������ ���ں��   
% % 
% % R1 = A1([1 2 3],[1 2 3]);
% % R2 = A2([1 2 3],[1 2 3]);
% % z0 = [0;0;1];
% % z1 = R1 * z0;
% % 
% % 
% % %% A1 �κ�
% % Number = 1
% % % A1�� �Ÿ�
% % Distance_A1_P0 = A1 ;
% % Distance_A1_P1 = A1 - A1;
% % 
% % % ���� �Ÿ� ���ϱ� ���꿡�� 4���ุ ��������, �� ����� X,Y,Z���� ������
% % A1_0 = Distance_A1_P0([1 2 3],[4]);  %�������� �� ��� A1*A2*A3*A4*A5*A6*A7�� ��ǥ�� - ��� A0�� ��ǥ��
% % A1_1 = Distance_A1_P1([1 2 3],[4]);  %�̰� A1*A2*A3*A4*A5*A6*A7 - A1  �� ��ġ��ǥ��
% % 
% % % ���ں�� ���� ���Ѱ�
% % A1_J1 =[cross(z0,A1_0);z0];
% % A1_J2 =[cross(z1,A1_1);z1];
% % 
% % Pre_Jaco_A1 = [A1_J1 A1_J2];
% % for i = 1:DH_Num
% % Jaco_A{Number, i} = Pre_Jaco_A1([1 2], :)*mid_point(i);
% % Check=Jaco_A{Number, i}
% % end
% % Number = Number +1
% % %% A1*A2 �κ�
% % 
% % % A1�� �Ÿ�
% % Distance_A12_P0 = A12 ;
% % Distance_A12_P1 = A12 - A1;
% % 
% % % ���� �Ÿ� ���ϱ� ���꿡�� 4���ุ ��������, �� ����� X,Y,Z���� ������
% % A12_0 = Distance_A12_P0([1 2 3],[4]);  %�������� �� ��� A1*A2*A3*A4*A5*A6*A7�� ��ǥ�� - ��� A0�� ��ǥ��
% % A12_1 = Distance_A12_P1([1 2 3],[4]);  %�̰� A1*A2*A3*A4*A5*A6*A7 - A1  �� ��ġ��ǥ��
% % 
% % % ���ں�� ���� ���Ѱ�
% % A12_J1 =[cross(z0,A12_0);z0];
% % A12_J2 =[cross(z1,A12_1);z1];
% % 
% % Pre_Jaco_A12 = [A12_J1 A12_J2];
% % Jaco_A12 = Pre_Jaco_A12([1 2], :)
% % 
% % for i=1:2
% %    Jaco_A{Number,i}=((Jaco_A12-Jaco_A{Number-1, DH_Num})*mid_point(i) ) +Jaco_A{Number-1, DH_Num}
% % end
% % 
% % for j =1 : DH_Num
% %     for i =1 : DH_Num
% %         Jaco_A{j,i}
% %     end
% % end
% % 

end

