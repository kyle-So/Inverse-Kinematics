function [ Inv_Jaco ] = So_Inv_Jacobian_for_Cal(Theta1, Theta2, Theta3, Theta4, Theta5, Theta6 , Theta7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  )
%SO_INV_JACOBIAN_CAL Summary of this function goes here
%   Detailed explanation goes here

t1 = Theta1;
t2 = Theta2;
t3 = Theta3;
t4 = Theta4;
t5 = Theta5;
t6 = Theta6;
t7 = Theta7;

% 7�� ���ں��
J=So_Show_Jacobian_form_DH(t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  );
% In_J_Cal_ing=So_Show_Jacobian_form_DH(t1,t2,t3,t4,t5,t6,t7, d1,d2,d3,d4,d5,d6,d7 , a1,a2,a3,a4,a5,a6,a7,  alpha1, alpha2,alpha3,alpha4,alpha5,alpha6,alpha7  )

% 7���̸� ���ں���� 6*7 ����̹Ƿ� ������� �Ұ�
% ���� J*J' * inv(J*J') = I �� ������ �̿� ��, J'�� J�� ��ġ����� Transpose���
 
New_Inv_J = J' * inv(J*J');



Inv_Jaco = New_Inv_J;
% In_Inv_Jaco=New_Inv_J
% Check_Inv_J= New_Inv_J*J;
end

