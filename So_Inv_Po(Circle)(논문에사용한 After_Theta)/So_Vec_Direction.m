function [Current_Diff_XYZ , Pre_Current_Diff_XYZ, Diff_Two_XYZ] = So_Vec_Direction(History_X,History_Y,History_Z,Repeat )
%SO_VEC_DIRECTION Summary of this function goes here
%   Detailed explanation goes here

Repeat;
            if Repeat >= 4
                Current_Diff_X = History_X(:,Repeat)-History_X(:,Repeat-1);
                Current_Diff_Y = History_Y(:,Repeat)-History_Y(:,Repeat-1);
                Current_Diff_Z = History_Z(:,Repeat)-History_Z(:,Repeat-1);  

                Pre_Current_Diff_X = History_X(:,Repeat-1)-History_X(:,Repeat-2);
                Pre_Current_Diff_Y = History_Y(:,Repeat-1)-History_Y(:,Repeat-2);
                Pre_Current_Diff_Z = History_Z(:,Repeat-1)-History_Z(:,Repeat-2); 

                Diff_Two_X = History_X(:,Repeat)-History_X(:,Repeat-2);
                Diff_Two_Y = History_Y(:,Repeat)-History_Y(:,Repeat-2);
                Diff_Two_Z = History_Z(:,Repeat)-History_Z(:,Repeat-2); 

        %         Pre_Diff_Two_X = History_X(:,Repeat-1)-History_X(:,Repeat-3);
        %         Pre_Diff_Two_Y = History_Y(:,Repeat-1)-History_Y(:,Repeat-3);
        %         Pre_Diff_Two_Z = History_Z(:,Repeat-1)-History_Z(:,Repeat-3); 

                Current_Diff_XYZ = [Current_Diff_X;Current_Diff_Y;Current_Diff_Z];
                Pre_Current_Diff_XYZ = [Pre_Current_Diff_X;Pre_Current_Diff_Y;Pre_Current_Diff_Z];
                Diff_Two_XYZ = [Diff_Two_X;Diff_Two_Y;Diff_Two_Z];
        %         Pre_Diff_Two_XYZ = [Pre_Diff_Two_X;Pre_Diff_Two_Y;Pre_Diff_Two_Z];

%                 [Current_Diff_XYZ Pre_Current_Diff_XYZ];  % 이거 곱해서 음수가 나오면 부호가 바뀌는거
        %         [Pre_Diff_Two_XYZ Diff_Two_XYZ] %결과적으로 이 두값은 거의 같다 빼기 조건으로 못씀
                % 변화량의 부호가 변화고, -2의 차가 작은경우를 조건으로 쓰면?
            end

Current_Diff_XYZ=132
Pre_Current_Diff_XYZ=223
Diff_Two_XYZ=323

end

