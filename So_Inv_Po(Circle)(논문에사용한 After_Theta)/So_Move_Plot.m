function [ output_args ] = So_Move_Plot(X,Y,Z, Goal_X,Goal_Y,Goal_Z, Ave_X,Ave_Y,Ave_Z , F_Sum, Repeat, Eraser,Judge_End )
%SO_MOVE_PLOT Summary of this function goes here
%   Detailed explanation goes here

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Bound_con_2 Coeff_rep Try_Num L_end

    figure(1)

%     xlim([-600 600]);
%     ylim([-600 600]);
%     zlim([0 1200]);
    xlim([-400 600]);
    ylim([-400 600]);
    zlim([0 1000]);
%     xlim([-400 1000]);
%     ylim([-200 1000]);
%     zlim([-200 1000]);
%     xlim([4.76 4.96]);
%     ylim([4.64 4.79]);
%     zlim([-2 8]);
    hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')

    % ������ ���� ���
    
    if Repeat == 1 
        for i = 1 : DOF
               if i == 1
        %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %�����ϳ���
                   Last_Position_1 = plot3([0 Goal_X{i,L_end}],[0 Goal_Y{i,L_end}],[0 Goal_Z{i,L_end}], 'LineWidth', 2,'color','r');     %�������� 
               else
        %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
                   Last_Position_etc = plot3([Goal_X{i-1,L_end} Goal_X{i,L_end}],[Goal_Y{i-1,L_end} Goal_Y{i,L_end}],[Goal_Z{i-1,L_end} Goal_Z{i,L_end}], 'LineWidth', 2,'color','r') ;
               end
        end
    end
     
    % ������ ��� �Ķ������� �ڵ���
    if Judge_End == 1
        for i = 1 : DOF
               if i == 1
        %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %�����ϳ���
                   Last_Position_1 = plot3([0 Goal_X{i,L_end}],[0 Goal_Y{i,L_end}],[0 Goal_Z{i,L_end}], 'LineWidth', 2,'color','b');     %�������� 
               else
        %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
                   Last_Position_etc = plot3([Goal_X{i-1,L_end} Goal_X{i,L_end}],[Goal_Y{i-1,L_end} Goal_Y{i,L_end}],[Goal_Z{i-1,L_end} Goal_Z{i,L_end}], 'LineWidth', 2,'color','b') ;
               end
        end
    end
    
    % Judge_End�� 1�� �� ������ �׷��� �����
    
    
    
    
    % plot3(Object(1),Object(2),Object(3),'ro')

    for i = 1 : DOF
           if i == 1
    %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %�����ϳ���
               Ave_RGraph_L{i}=plot3([0 Ave_X{i,L_end}],[0 Ave_Y{i,L_end}],[0 Ave_Z{i,L_end}], 'LineWidth', 2);     %�������� 
           else
    %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
               Ave_RGraph_L{i}=plot3([Ave_X{i-1,L_end} Ave_X{i,L_end}],[Ave_Y{i-1,L_end} Ave_Y{i,L_end}],[Ave_Z{i-1,L_end} Ave_Z{i,L_end}], 'LineWidth', 2) ;
           end
    end
                Eraser = Eraser +1;

                
        if mod(Repeat,10) == 1          %10������
            for i = 1:2 : DOF
            %    for j = 1 : length(mid_point)  % �̰� mid_point�� ����
                    j = length(mid_point);      % �̰� �� ��ũ ���� ����ǥ��
%                   Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'k', 'LineWidth', 1);  %��� ǥ��
%                   Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %����ǥ��
        %           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
        %           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
            %    end
            end
        end
        
                    for i = 1:2 : DOF
%                        for j = 1 : length(mid_point)  % �̰� mid_point�� ����
                            j = length(mid_point);  % �̰� �� ��ũ ���� ����ǥ��
        %                   Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'k', 'LineWidth', 1);  %��� ǥ��
%                           Disp_Vec{i,j}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %����ǥ��
% % %                             Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %����ǥ�� �� ��ũ ����
                %           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
                %           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
%                        end
                     end

    if Repeat ==1
        pause(3)
    end

    pause(0.00002);

    % % 10������ ���� �׷����� ����� �������� ��� ������ ����� �츲
    % if Repeat == Try_Num
    %     for k = 1 : 9
    %         for i = 1 : DOF
    %              delete(Graph_L{k,i});
    %         end 
    %     end
    % else
    %     if Eraser == 11   
    %          pause(0.02);
    %         for i = 1 : DOF
    %              delete(Graph_L{:,i});
    %         end 
    %         Eraser = 1;
    %     end
    % end 

    % %     �̰� �ǽð����� ����°�
        if Repeat ~= Try_Num 
%             if mod(Repeat,100) ~= 0
            if Judge_End == 0
                for i = 1 : DOF
                         delete(Ave_RGraph_L{i});
                end
            end
            
            
            for i = 1 :2:DOF
%                  if mod(Repeat,10) == 1
%                      delete(Point_Dis{i});
% % %                      delete(Disp_Vec{i});
    %                  delete(Disp_Vatt{i});
    %                  delete(Disp_Vrep{i});                
%                 end 
            end
            
%             % ���Ͱ� mid_point �������� ����
%             for i = 1 : DOF
%                 for j = 1 : length(mid_point)
%                     delete(Disp_Vec{i,j});
%                 end
%             end
            
            
%             end
        end   




end

