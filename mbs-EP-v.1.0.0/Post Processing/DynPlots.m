function [] = DynPlots(GraphicsType,BodiesGraph,JointsGraph,RunTime,TimeStep,CoM,Points)
%This function is responsible for plotting the graphics for the dynamic
%simulation.

%% Inputs Treatment
posgraph = GraphicsType.posgraph;
travelgraph = GraphicsType.travelgraph;
angvelgraph = GraphicsType.angvelgraph;
traaccelgraph = GraphicsType.traaccelgraph;
angaccelgraph = GraphicsType.angaccelgraph;
comgraph = GraphicsType.comgraph;
pointsgraph = GraphicsType.pointsgraph;
a = size(CoM,2);
b = size(BodiesGraph,1);
c = size(Points,2);
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
            elseif strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
                    %Translational Velocity Info
                    travel = CoM{1,j}(nbod).TraVel;
                    xvel(1,j) = travel(1,1);
                    yvel(1,j) = travel(2,1);
                    zvel(1,j) = travel(3,1);
            elseif strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
                    %Angular Velocity Info
                    angvel = CoM{1,j}(nbod).AngVel;
                    wxvel(1,j) = angvel(1,1);
                    wyvel(1,j) = angvel(2,1);
                    wzvel(1,j) = angvel(3,1);
            elseif strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
                    %Translational Acceleration Info
                    traaccel = CoM{1,j}(nbod).TraAccel;
                    xaccel(1,j) = traaccel(1,1);
                    yaccel(1,j) = traaccel(2,1);
                    zaccel(1,j) = traaccel(3,1);
            elseif strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
                    angaccel = CoM{1,j}(nbod).AngAccel;
                    wxaccel(1,j) = angaccel(1,1);
                    wyaccel(1,j) = angaccel(2,1);
                    wzaccel(1,j) = angaccel(3,1);
            end
        end

        %% Lines of Code Responsible for the Plots
        if strcmp(posgraph,'Yes') == 1 || strcmp(posgraph,'yes') == 1
            % Position Plots
            figure('Name','Positions','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xpos,'r');
            title('Position in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,ypos,'b');
            title('Position in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zpos,'k');
            title('Position in z');

        elseif strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
            % Translational Velocity Plots
            figure('Name','Translational Velocities','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xavisvec,xvel,'r');
            title('Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yvel,'b');
            title('Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zvel,'k');
            title('Velocity in z');
        elseif strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
            % Angular Velocity Plots
            figure('Name','Angular Velocities','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxvel,'r');
            title('Angular Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyvel,'b');
            title('Angular Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzvel,'k');
            title('Angular Velocity in z');

        elseif strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
            % Translational Acceleration Plots
            figure('Name','Translational Accelerations','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xaccel,'r');
            title('Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yaccel,'b');
            title('Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zaccel,'k');
            title('Acceleration in z')
        elseif strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
            % Angular Acceleration Plots
            figure('Name','Angular Accelerations','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxaccel,'r');
            title('Angular Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyaccel,'b');
            title('Angular Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzaccel,'k');
            title('Angular Acceleration in z')
        end     
    end
%% Plots Points Info    
elseif strcmp(pointsgraph,'Yes') == 1 || strcmp(pointsgraph,'yes') == 1 || strcmp(pointsgraph,'YES') == 1
    %% Plots Points Information
    for i = 1:c
        nbod = JointsGraph{i,1};
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
                    pos = Points{1,j}(nbod).Position;
                    xpos(1,j) = pos(1,1);
                    ypos(1,j) = pos(2,1);
                    zpos(1,j) = pos(3,1);
            elseif strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
                    %Translational Velocity Info
                    travel = Points{1,j}(nbod).TraVel;
                    xvel(1,j) = travel(1,1);
                    yvel(1,j) = travel(2,1);
                    zvel(1,j) = travel(3,1);
            elseif strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
                    %Angular Velocity Info
                    angvel = Points{1,j}(nbod).AngVel;
                    wxvel(1,j) = angvel(1,1);
                    wyvel(1,j) = angvel(2,1);
                    wzvel(1,j) = angvel(3,1);
            elseif strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
                    %Translational Acceleration Info
                    traaccel = Points{1,j}(nbod).TraAccel;
                    xaccel(1,j) = traaccel(1,1);
                    yaccel(1,j) = traaccel(2,1);
                    zaccel(1,j) = traaccel(3,1);
            elseif strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
                    angaccel = Points{1,j}(nbod).AngAccel;
                    wxaccel(1,j) = angaccel(1,1);
                    wyaccel(1,j) = angaccel(2,1);
                    wzaccel(1,j) = angaccel(3,1);
            end
        end

        %% Lines of Code Responsible for the Plots
        if strcmp(posgraph,'Yes') == 1 || strcmp(posgraph,'yes') == 1
            % Position Plots
            figure('Name','Points Positions','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xpos,'r');
            title('Position in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,ypos,'b');
            title('Position in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zpos,'k');
            title('Position in z');

        elseif strcmp(travelgraph,'Yes') == 1 || strcmp(travelgraph,'yes') == 1
            % Translational Velocity Plots
            figure('Name','Points Translational Velocities','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xavisvec,xvel,'r');
            title('Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yvel,'b');
            title('Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zvel,'k');
            title('Velocity in z');
        elseif strcmp(angvelgraph,'Yes') == 1 || strcmp(angvelgraph,'yes') == 1
            % Angular Velocity Plots
            figure('Name','Points Angular Velocities','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxvel,'r');
            title('Angular Velocity in x');
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyvel,'b');
            title('Angular Velocity in y');
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzvel,'k');
            title('Angular Velocity in z');

        elseif strcmp(traaccelgraph,'Yes') == 1 || strcmp(traaccelgraph,'yes') == 1
            % Translational Acceleration Plots
            figure('Name','Points Translational Accelerations','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,xaccel,'r');
            title('Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,yaccel,'b');
            title('Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,zaccel,'k');
            title('Acceleration in z')
        elseif strcmp(angaccelgraph,'Yes') == 1 || strcmp(angaccelgraph,'yes') == 1
            % Angular Acceleration Plots
            figure('Name','Points Angular Accelerations','NumberTitle','off');
            subplot(3,1,1)
            axis auto
            plot(xaxisvec,wxaccel,'r');
            title('Angular Acceleration in x')
            subplot(3,1,2)
            axis auto
            plot(xaxisvec,wyaccel,'b');
            title('Angular Acceleration in y')
            subplot(3,1,3)
            axis auto
            plot(xaxisvec,wzaccel,'k');
            title('Angular Acceleration in z')
        end     
    end   
end
end

