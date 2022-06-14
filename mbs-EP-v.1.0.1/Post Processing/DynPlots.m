function [] = DynPlots(GraphicsType,BodiesGraph,PointsGraph,RunTime,TimeStep,CoM,Points)
%This function is responsible for plotting the graphics for the dynamic
%simulation.

%% Inputs Treatment
posgraph = GraphicsType.posgraph{1,1};
travelgraph = GraphicsType.travelgraph{1,1};
angvelgraph = GraphicsType.angvelgraph{1,1};
traaccelgraph = GraphicsType.traaccelgraph{1,1};
angaccelgraph = GraphicsType.angaccelgraph{1,1};
comgraph = GraphicsType.comgraph{1,1};
pointsgraph = GraphicsType.pointsgraph{1,1};
a = size(CoM,2);
b = size(BodiesGraph,1);
c = size(Points,2);
d = size(PointsGraph,2);
xaxisvec = 0:TimeStep:RunTime;

%% Retrieval and Plot information
if strcmp(comgraph,'Yes') == 1 || strcmp(comgraph,'yes') == 1 || strcmp(comgraph,'YES') == 1
%% Plots CoM Information
    for i = 1:b
        nbod = BodiesGraph{i,1};
        %% Pre Allocation of Vectors.
        xpos = zeros(1,a);
        ypos = zeros(1,a);
        zpos = zeros(1,a);
        xvel = zeros(1,a);
        yvel = zeros(1,a);
        zvel = zeros(1,a);
        wxvel = zeros(1,a);
        wyvel = zeros(1,a);
        wzvel = zeros(1,a);
        xaccel = zeros(1,a);
        yaccel = zeros(1,a);
        zaccel = zeros(1,a);
        wxaccel = zeros(1,a);
        wyaccel = zeros(1,a);
        wzaccel = zeros(1,a);

        %% Retrieving the plot y axis from the data cell structs
        for j = 1:a
            if strcmp(posgraph,'Yes') == 1 || strcmp(posgraph,'yes') == 1
                    %Position Info
                    pos = CoM{1,j}(nbod).Position;
                    xpos(1,j) = pos(1,1);
                    ypos(1,j) = pos(2,1);
                    zpos(1,j) = pos(3,1);
            end
            if strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
                    %Translational Velocity Info
                    travel = CoM{1,j}(nbod).TraVel;
                    xvel(1,j) = travel(1,1);
                    yvel(1,j) = travel(2,1);
                    zvel(1,j) = travel(3,1);
            end
            if strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
                    %Angular Velocity Info
                    angvel = CoM{1,j}(nbod).AngVel;
                    wxvel(1,j) = angvel(1,1);
                    wyvel(1,j) = angvel(2,1);
                    wzvel(1,j) = angvel(3,1);
            end
            if strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
                    %Translational Acceleration Info
                    traaccel = CoM{1,j}(nbod).TraAccel;
                    xaccel(1,j) = traaccel(1,1);
                    yaccel(1,j) = traaccel(2,1);
                    zaccel(1,j) = traaccel(3,1);
            end
            if strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
                    angaccel = CoM{1,j}(nbod).AngAccel;
                    wxaccel(1,j) = angaccel(1,1);
                    wyaccel(1,j) = angaccel(2,1);
                    wzaccel(1,j) = angaccel(3,1);
            end
        end

        %% Lines of Code Responsible for the Plots
        if strcmp(posgraph,'Yes') == 1 || strcmp(posgraph,'yes') == 1
            % Position Plots
            str = 'CoM:Position Body ' + string(nbod);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xpos,'r');
            title('CoM Position in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,ypos,'b');
            title('CoM Position in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zpos,'k');
            title('CoM Position in z');
        end

        if strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
            % Translational Velocity Plots
            str = 'CoM:Translational Velocities ' + string(nbod);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xvel,'r');
            title('CoM Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yvel,'b');
            title('CoM Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zvel,'k');
            title('CoM Velocity in z');
        end
        if strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
            % Angular Velocity Plots
            str = 'CoM:Angular Velocities ' + string(nbod);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxvel,'r');
            title('CoM Angular Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyvel,'b');
            title('CoM Angular Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzvel,'k');
            title('CoM Angular Velocity in z');
        end

        if strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
            % Translational Acceleration Plots
            str = 'CoM:Translational Accelerations ' + string(nbod);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xaccel,'r');
            title('CoM Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yaccel,'b');
            title('CoM Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zaccel,'k');
            title('CoM Acceleration in z')
        end
        if strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
            % Angular Acceleration Plots
            str = 'CoM:Angular Accelerations ' + string(nbod);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxaccel,'r');
            title('CoM Angular Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyaccel,'b');
            title('CoM Angular Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzaccel,'k');
            title('CoM Angular Acceleration in z')
        end     
    end
%% Plots Points Info    
elseif strcmp(pointsgraph,'Yes') == 1 || strcmp(pointsgraph,'yes') == 1 || strcmp(pointsgraph,'YES') == 1
    %% Plots Points Information
    for i = 1:d
        npoint = PointsGraph{i,1};
        %% Pre Allocation of Vectors.
        xpos = zeros(1,c);
        ypos = zeros(1,c);
        zpos = zeros(1,c);
        xvel = zeros(1,c);
        yvel = zeros(1,c);
        zvel = zeros(1,c);
        wxvel = zeros(1,c);
        wyvel = zeros(1,c);
        wzvel = zeros(1,c);
        xaccel = zeros(1,c);
        yaccel = zeros(1,c);
        zaccel = zeros(1,c);
        wxaccel = zeros(1,c);
        wyaccel = zeros(1,c);
        wzaccel = zeros(1,c);

        %% Retrieving the plot y axis from the data cell structs
        for j = 1:c
            if strcmp(posgraph,'Yes') == 1 || strcmp(posgraph,'yes') == 1
                    %Position Info
                    pos = Points{1,j}(npoint).Position;
                    xpos(1,j) = pos(1,1);
                    ypos(1,j) = pos(2,1);
                    zpos(1,j) = pos(3,1);
            end
            if strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
                    %Translational Velocity Info
                    travel = Points{1,j}(npoint).TraVel;
                    xvel(1,j) = travel(1,1);
                    yvel(1,j) = travel(2,1);
                    zvel(1,j) = travel(3,1);
            end
            if strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
                    %Angular Velocity Info
                    angvel = Points{1,j}(npoint).AngVel;
                    wxvel(1,j) = angvel(1,1);
                    wyvel(1,j) = angvel(2,1);
                    wzvel(1,j) = angvel(3,1);
            end
            if strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
                    %Translational Acceleration Info
                    traaccel = Points{1,j}(npoint).TraAccel;
                    xaccel(1,j) = traaccel(1,1);
                    yaccel(1,j) = traaccel(2,1);
                    zaccel(1,j) = traaccel(3,1);
            end
            if strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
                    angaccel = Points{1,j}(npoint).AngAccel;
                    wxaccel(1,j) = angaccel(1,1);
                    wyaccel(1,j) = angaccel(2,1);
                    wzaccel(1,j) = angaccel(3,1);
            end
        end

        %% Lines of Code Responsible for the Plots
        if strcmp(posgraph,'Yes') == 1 || strcmp(posgraph,'yes') == 1
            % Position Plots
            str = 'Points:Position Point ' + string(npoint);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xpos,'r');
            title('Point Position in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,ypos,'b');
            title('Point Position in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zpos,'k');
            title('Point Position in z');
        end

        if strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
            % Translational Velocity Plots
            str = 'Points:Translational Velocity Point ' + string(npoint);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xavisvec,xvel,'r');
            title('Point Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yvel,'b');
            title('Point Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zvel,'k');
            title('Point Velocity in z');
        end
        if strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
            % Angular Velocity Plots
            str = 'Points:Angular Velocity Point ' + string(npoint);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxvel,'r');
            title('Point Angular Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyvel,'b');
            title('Point Angular Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzvel,'k');
            title('Point Angular Velocity in z');
        end
        if strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
            % Translational Acceleration Plots
            str = 'Points:Translational Acceleration Point ' + string(npoint);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xaccel,'r');
            title('Point Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yaccel,'b');
            title('Point Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zaccel,'k');
            title('Point Acceleration in z')
        end
        if strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
            % Angular Acceleration Plots
            str = 'Points:Angular Acceleration Point ' + string(npoint);
            figure('Name',str,'NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxaccel,'r');
            title('Point Angular Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyaccel,'b');
            title('Point Angular Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzaccel,'k');
            title('Point Angular Acceleration in z')
        end     
    end   
end
end

