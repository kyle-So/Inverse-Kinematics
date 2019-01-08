L1 = 3, L2 = 4

X=4,Y=0

Theta1 = atand(Y/X) - acosd( (X^2+Y^2+L1^2-L2^2) / (2*L1*sqrt(X^2+Y^2)   )  )

Theta2 = 180- acosd(  (  L1^2+L2^2-  (X^2+Y^2) )    / (2*L1*L2)  )


Result_X = 3*cosd(Theta1) + 4*cosd(Theta1+Theta2)
Result_Y = 3*sind(Theta1) + 4*sind(Theta1+Theta2)
