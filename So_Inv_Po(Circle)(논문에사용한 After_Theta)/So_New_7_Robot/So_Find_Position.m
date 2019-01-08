clc
clear

syms t0 t1 t2 t3 t4 t5 t6 t7
syms d0 d1 d2 d3 d4 d5 d6 d7
syms a0 a1 a2 a3 a4 a5 a6 a7
syms nx ny nz ox oy oz ax ay az px py pz x1 x2
syms f1 f2 X Y Z

%% DH�Ķ���� ��

% ��Ÿ�� �����϶�, �׸��� t123456 �� �ʱⰪ���� �Է��Ѵ�.
% DH �ʱⰪ ��, ��Ÿ �ʱⰪ
%******************************************************
t1     ; d1=0          ; a1=0         ;  alpha1 =  90;
t2     ; d2=0          ; a2=1           ;  alpha2 =   0;
t3     ; d3=0          ; a3=1           ;  alpha3 =   0;
t4    ; d4=0          ; a4=1           ;  alpha4 = -90;
t5     ; d5=0          ; a5=0         ;  alpha5 =  90;
t6     ; d6=0          ; a6=0         ;  alpha6 =   0;
%******************************************************
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
   
Sum_T=(A1*A2*A3*A4*A5*A6);

% �ʱ������ ��������� ����. �ϴ� �̰� �� DH ��Ÿ�� ���� ������ -> �ϴ� �ʱⰪ����
Position_X =  cos(t1)*(a3*cos(t2 + t3) + a2*cos(t2) + a4*cos(t2 + t3 + t4));
Position_Y =  sin(t1)*(a3*cos(t2 + t3) + a2*cos(t2) + a4*cos(t2 + t3 + t4));
Position_Z =  a3*sin(t2 + t3) + a2*sin(t2) + a4*sin(t2 + t3 + t4);

% �ʱ� ���� XYZ�࿡ ���� NOA
NOA([1 2 3],[1 2 3]) = Sum_T([1 2 3],[1 2 3]);

Ini_Delta_Z_A = atan2(  NOA(2,1) , NOA(1,1) );
Ini_Delta_Y_O = atan2( -NOA(3,1) , NOA(1,1)*cos(Ini_Delta_Z_A)+NOA(2,1)*sin(Ini_Delta_Z_A) );
Ini_Delta_X_N = atan2( -NOA(2,3)*cos(Ini_Delta_Z_A)+NOA(1,3)*sin(Ini_Delta_Z_A), NOA(2,2)*cos(Ini_Delta_Z_A)-NOA(1,2)*sin(Ini_Delta_Z_A));

Rotate_X = So_Rad_to_Deg( Ini_Delta_X_N);
Rotate_Y = So_Rad_to_Deg( Ini_Delta_Y_O);
Rotate_Z = So_Rad_to_Deg( Ini_Delta_Z_A);

% ������� ���ⱸ�� ��ġ�ؼ�����
%% �̰ɷ� �����ؼ� ����� ��Ȯ�ϸ� ���� ���ں�� �ŷڰ��� �� �����ں�� ��� �غ�

section = 100; %��м� ex) 10��� => 11���� ������



%��ġ,���� ��ģ��
Position_Theta = [Position_X; Position_Y ; Position_Z ; Ini_Delta_X_N ; Ini_Delta_Y_O ;Ini_Delta_Z_A ];


%��ǥ�� �Է� - > 
%%%%%%%%%%%%%%%%%%%%% ����� ���� �Է����� �ϴ� ��ü
%��ǥ�� �Է±���
Theta1 = pi/2 *0  ;
Theta2 = pi/2  ;
Theta3 = pi/2*0  ;
Theta4 = pi/3  ;
Theta5 = pi/2*0  ;
Theta6 = pi/2*0  ;

%%%%%%%%%%%%%%%%%%%%%
Object_Theta([1 2 3 4 5 6],1) = [Theta1 ; Theta2 ; Theta3 ; Theta4 ; Theta5 ; Theta6]; %  <- �ϴ� ���������Է�

So_SumT_for_Cal(Theta1,Theta2,Theta3,Theta4,Theta5,Theta6)

d_Theta([1;2;3;4;5;6],1) = (Object_Theta - [t1;t2;t3;t4;t5;t6])/section 

So_T([1 2 3 4 5 6],1)= [t1;t2;t3;t4;t5;t6];


%���ⱸ�� ���
  
  for i=1 :section
      So_T([1 2 3 4 5 6],i+1) = So_T([1 2 3 4 5 6],i)+(d_Theta)   ;               %�ʱ���� ��ǥġ���� ��Ÿ
      [Jaco]=So_Jacobian_for_Cal(So_T(1,i),So_T(2,i),So_T(3,i),So_T(4,i),So_T(5,i),So_T(6,i),a2,a3,a4);
      dP = Jaco * d_Theta
      Position_Theta = Position_Theta + dP
  end
  PPANS=[Position_Theta([1 2 3],1);Position_Theta([4 5 6],1)*180/pi]
So_T([1 2 3 4 5 6],section+1);
