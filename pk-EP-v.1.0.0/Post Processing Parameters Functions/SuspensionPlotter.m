function SuspensionPlotter(Points,Points_before, to_hold)

arguments
    Points
    Points_before
    to_hold = 0
end

%INPUTS: -Points: cell array with the location of each point along the
%various iterations
%
%OUTPUTS:-Plot with the suspension points and members

%Scope: Take the points of the suspension and represent them in an
%graphical fashion



for i=1:size(Points,1)
    for j=1:size(Points,2)
        first_point_array(i,j)=Points{i,j}(1);
        middle_point_array(i,j)=Points{i,j}(floor(end/2));
        last_point_array(i,j)=Points{i,j}(end);
    end
end


%points to be connected:
%FRONT
%Upper A-Arm Left
%1-3
%2-3
%Lower A-Arm Left
%4-6
%5-6
%Tie-Rod Left
%7-8
%Upright Left
%3-6
%3-7
%6-7
%Upper A-Arm Right
%13-15
%14-15
%Lower A-Arm Right
%16-18
%17-18
%Tie-Rod Right
%19-20
%Upright Right
%15-18
%15-19
%18-19
%Rack
%8-20
%REAR
%front points+24;
conect=[1 3; 2 3; 4 6; 5 6; 7 8; 3 6; 3 7; 6 7; 13 15; 14 15; 16 18; 17 18;...
    19 20; 15 18; 15 19; 18 19; 8 20; 25 27; 26 27; 28 30; 29 30; 31 32;...
    27 30; 27 31; 30 31; 37 39; 38 39; 40 42; 41 42; 43 44; 39 42; 39 43;...
    42 43; 49 50; 54 55; 59 60; 64 65; 50 51; 51 52; 50 52; 55 56; 56 57;...
    55 57; 60 61; 61 62; 60 62; 65 66; 66 67; 65 67; 52 53; 57 58; 62 63;...
    67 68]; %conectivities matrix, showing all the points to be connected

if to_hold == 1
    
else
    figure
    
end

hold on
axis equal

for i=1:length(conect)
    
    duplo = cell2mat(Points_before);
    point1= duplo(conect(i,1),:);
    point2= duplo(conect(i,2),:);
    coordinates=[point1;point2];
    plot3(coordinates(:,1),coordinates(:,2),coordinates(:,3),'-sk','Color','red')
    
end

current_axes = gca;
current_axes.View = [69.7608   10.63079];

% for very nice cool gif
axis manual

clear point1 point2 coordinates

for i = 1:length(Points{1,1})
    
    array=Gen_array_Points(Points,i);
    cla
    
    for j=1:length(conect)
        
        point1= array(conect(j,1),:);
        point2= array(conect(j,2),:);
        coordinates=[point1;point2];
        
        plot3(coordinates(:,1),coordinates(:,2),coordinates(:,3),...
            '-sk','Color','green')
        
        
        
    end
    
    if 1
        drawnow;
        %pause(0.2)
    end
    
    
end



