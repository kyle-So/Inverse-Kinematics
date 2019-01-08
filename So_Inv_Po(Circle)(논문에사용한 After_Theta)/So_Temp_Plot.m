function [ output_args ] = So_Temp_Plot(X,Y,Z, Goal_X,Goal_Y,Goal_Z,F_Sum, Repeat, Eraser )
%SO_PLOT_ORIGIN Summary of this function goes here
%   Detailed explanation goes here

global d a alpha mid_point Obstacle Obs_Circle Obs_Cir_Radius DOF Coeff_att Bound_con Bound_con_2 Coeff_rep Try_Num L_end


    figure(1)
%     xlim([-400 400]);
%     ylim([-400 400]);
%     zlim([0 800]);
    xlim([-2 10]);
    ylim([-2 10]);
    zlim([-2 8]);
%     xlim([4.76 4.96]);
%     ylim([4.64 4.79]);
%     zlim([-2 8]);
    hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')

    % 마지막 형태 출력
    for i = 1 : DOF
           if i == 1
    %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %둘중하나는
               plot3([0 Goal_X{i,L_end}],[0 Goal_Y{i,L_end}],[0 Goal_Z{i,L_end}], 'LineWidth', 2,'color','r');     %지워야함 
           else
    %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
               plot3([Goal_X{i-1,L_end} Goal_X{i,L_end}],[Goal_Y{i-1,L_end} Goal_Y{i,L_end}],[Goal_Z{i-1,L_end} Goal_Z{i,L_end}], 'LineWidth', 2,'color','r') ;
           end
    end

    % plot3(Object(1),Object(2),Object(3),'ro')

    for i = 1 : DOF
           if i == 1
    %            Graph_L{Eraser,i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);  %둘중하나는
               RGraph_L{i}=plot3([0 X{i,L_end}],[0 Y{i,L_end}],[0 Z{i,L_end}], 'LineWidth', 2);     %지워야함 
           else
    %            Graph_L{Eraser,i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
               RGraph_L{i}=plot3([X{i-1,L_end} X{i,L_end}],[Y{i-1,L_end} Y{i,L_end}],[Z{i-1,L_end} Z{i,L_end}], 'LineWidth', 2) ;
           end
    end
                Eraser = Eraser +1;

    for i = 1 : DOF
    %    for j = 1 : length(mid_point)
            j = length(mid_point);
%           Point_Dis{i}=plot3(X{i,j},Y{i,j},Z{i,j},'.' ,'Color', 'c', 'LineWidth', 1);  %경로 표시
%           Disp_Vec{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Sum{i,j}(1),F_Sum{i,j}(2),F_Sum{i,j}(3),'LineWidth', 2);  %벡터표시
%           Disp_Vatt{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vatt{i,j}(1),F_Vatt{i,j}(2),F_Vatt{i,j}(3),'LineWidth', 1);
%           Disp_Vrep{i}=quiver3(X{i,j},Y{i,j},Z{i,j},F_Vrep{i,j}(1),F_Vrep{i,j}(2),F_Vrep{i,j}(3),'LineWidth', 1);
    %    end
    end

    if Repeat ==1
        pause(3)
    end

    pause(0.00002);

    % % 10개마다 이전 그래프를 지우고 마지막일 경우 마지막 모양은 살림
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

    % %     이건 실시간으로 지우는거
        if Repeat ~= Try_Num 
%             if mod(Repeat,100) ~= 0
                for i = 1 : DOF
                     delete(RGraph_L{i});
%                      delete(Point_Dis{i});
%                      delete(Disp_Vec{i});
    %                  delete(Disp_Vatt{i});
    %                  delete(Disp_Vrep{i});                
                end 
%             end
        end

end

