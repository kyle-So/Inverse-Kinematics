%% 이 파일은 So_Jacobian의 7축 코드로 
%  포텐셜필드가 X,Y,Z 힘만 요구하므로 오리엔테이션을 지운 파일을 사용했기에 비상용으로 원본파일을 남겨둠



function [ Use_Jaco ] = So_Jacobian( Theta )      

global d a alpha mid_point

    t= Theta;
%******************************************************
    
Number_of_Link = length(t);         %전체의 링크 수, 2축예니까 2라 적음
for i_Link = 1 : Number_of_Link
%     disp('전체계산 시작함')

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
    
% 초기값 설정 부분 회전부분, 그리고 자코비안 계산하기위한 현재의 x,y,z부분 등등 초기값
    z0 = [0;0;1];
    Z(:,:,1)=[0;0;1];
    Distance_A(:,:,1) = Times_A(:,:,DH_Num);      %초기값 설정
    Sum_J{i_Link}=[];
    Sum_Z{i_Link}=[];
% 기존의 자코비안 계산 부분, 속도의 경우 외적 z,위치부분을 했고,
% 오리엔테이션의 경우 R1*..R(n-1)* [0;0;1] 하면 됨
% 물리적으로 생각하면 당연한 소리. 자세한건 한글 자코비안 정리파일 참고(알고 쓰는거지만 혹시 잊어버렸으면..)
% 그래서 결론적으로 이부분은 자코비안을 구하는건데 mid_point를 써야하므로 속도, 각속도 부분을 나눔
    for i = 1 : DH_Num
%         disp('같은 자코비안 계산중')
         R(:,:,i) = Times_A([1 2 3],[1 2 3],i) ;  % 예를들어 R2 는 R1*R2를 뜻함
         Z(:,:,i+1) = R(:,:,i)*[0;0;1]  ;         % 자코비안 오리엔테이션값
         Distance_A(:,:,i+1) = Times_A(:,:,DH_Num)-Times_A(:,:,i);
         Position_A(:,:,i) = Distance_A([1 2 3],[4],i);   % mid_point는 회전은 상관없음
         J(:,:,i) = [cross(Z(:,:,i),Position_A(:,:,i)) ; Z(:,:,i)];  % 이걸 쓰면 속도,회전 자코비안 두개 다 얻을수 있음
%                                                                      %하지만 mid_point의 값 계산이 필요하므로 속도부분만 따로 계산 
         Vel_J(:,:,i) = [cross(Z(:,:,i),Position_A(:,:,i))];    % 포지션(속도)부분만 계산
         
         Sum_J{i_Link} = [Sum_J{i_Link} Vel_J(:,:,i)] ;                                   
         Sum_Z{i_Link} = [Sum_Z{i_Link} Z(:,:,i)] ;
%          PREPREPREPRE =Sum_J{i_Link}

    end
    
             
         if DH_Num < Number_of_Link
%              disp('조건문들어옴')
             Sum_J{i_Link} = [Sum_J{i_Link} zeros(3,Number_of_Link-DH_Num)];
             Sum_Z{i_Link} = [Sum_Z{i_Link} zeros(3,Number_of_Link-DH_Num)];
%              RRRR=Sum_J{i_Link}
         else
%              disp('엘스들어옴')
             Sum_J{i_Link};
             Sum_Z{i_Link};
%              EEEE=Sum_J{i_Link}
         end
    
    
    
%     disp('검산들어옴')
    Sum_J{:};   % 검산용 나중에 주석처리
                 % 결국 자코비안 구조체 처리를 위한 1축 A1 ,2축 A12 ,3축 A123 등의 자코비안 구한값
                 % 아래에서 이제 구조체의 오른쪽을 만들거임.
    

% disp('구조체 정리부분')
% i_Link = 현재의 링크 즉 A1, A12, A123 링크중 만약 C_Link=3 이라면 A123을 쓰고있는거임
      
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

% disp('요약')
Use_Jaco{:,:};






% Structure_Jaco{2,2}
% cellplot(Structure_Jaco)

% % 
% % 
% % %% 기존 방법
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
% % %% 자코비안 출력하기
% % 
% % A12 = A1*A2
% % % 상수 일때 이거  
% % % Position_A1 = A1([1 2],4);
% % 
% % %% 심볼릭일때 요거   
% % 
% % % P_A1 = simplify(A1([1 2],4))
% % % Jaco_A1 = simplify(jacobian(P_A1,[t1,t2]))
% % % 
% % % P_A12 = simplify(A12([1 2],4))
% % % Jaco_A12 = simplify(jacobian(P_A12,[t1,t2]))
% % %% 손으로 자코비안   
% % 
% % R1 = A1([1 2 3],[1 2 3]);
% % R2 = A2([1 2 3],[1 2 3]);
% % z0 = [0;0;1];
% % z1 = R1 * z0;
% % 
% % 
% % %% A1 부분
% % Number = 1
% % % A1의 거리
% % Distance_A1_P0 = A1 ;
% % Distance_A1_P1 = A1 - A1;
% % 
% % % 위에 거리 구하기 연산에서 4열행만 가져오기, 즉 행렬중 X,Y,Z값만 가져옴
% % A1_0 = Distance_A1_P0([1 2 3],[4]);  %변수명의 뜻 행렬 A1*A2*A3*A4*A5*A6*A7의 좌표값 - 행렬 A0의 좌표값
% % A1_1 = Distance_A1_P1([1 2 3],[4]);  %이건 A1*A2*A3*A4*A5*A6*A7 - A1  의 위치좌표값
% % 
% % % 자코비안 각각 구한거
% % A1_J1 =[cross(z0,A1_0);z0];
% % A1_J2 =[cross(z1,A1_1);z1];
% % 
% % Pre_Jaco_A1 = [A1_J1 A1_J2];
% % for i = 1:DH_Num
% % Jaco_A{Number, i} = Pre_Jaco_A1([1 2], :)*mid_point(i);
% % Check=Jaco_A{Number, i}
% % end
% % Number = Number +1
% % %% A1*A2 부분
% % 
% % % A1의 거리
% % Distance_A12_P0 = A12 ;
% % Distance_A12_P1 = A12 - A1;
% % 
% % % 위에 거리 구하기 연산에서 4열행만 가져오기, 즉 행렬중 X,Y,Z값만 가져옴
% % A12_0 = Distance_A12_P0([1 2 3],[4]);  %변수명의 뜻 행렬 A1*A2*A3*A4*A5*A6*A7의 좌표값 - 행렬 A0의 좌표값
% % A12_1 = Distance_A12_P1([1 2 3],[4]);  %이건 A1*A2*A3*A4*A5*A6*A7 - A1  의 위치좌표값
% % 
% % % 자코비안 각각 구한거
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

