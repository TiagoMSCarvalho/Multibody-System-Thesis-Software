function [] = Plotter(Points,Joints,it)
%Plots the data of the mechanism in 3D and calls the pre made gif function.
%% Data Retrievel from the Points Cell Arrays.    
    k = 1;
    a = 1;
    j = 1;
    ite = it-1;
    for i = 1:ite
        Pos1 = Points{1,i};
        for k = 1:Joints.NPoint
            Pos2(a,:) = Pos1(k).Position;
            a = a+1;
        end
    end
%% Plot of the Information regarding the Position of the points at each iteration    
    figure('Name','Position per Iteration','NumberTitle','off');
    hold on; 
    grid on;
    for i = 1:ite
        b = Joints.NPoint*i;
        for j = j:(b-1) 
        plot3([Pos2(j,1) Pos2(j+1,1)],[Pos2(j,2) Pos2(j+1,2)],[Pos2(j,3) Pos2(j+1,3)],'-or');
        end
        j = j + Joints.NPoint;
    end
    hold off;
    xlabel('x axis [mm]');
    ylabel('y axis [mm]');
    zlabel('z axis [mm]');
    axis tight;
    view(3);
end

