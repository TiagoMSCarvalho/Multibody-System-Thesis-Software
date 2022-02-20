function [Points,it] = KinDataStorage(Points,Bodies,Joints,vel,acc,it)
%Store data from each iteration regarding the points to be study.

    %% Creates a struct name Data to store Point Position, Vel, Acc
    Data = struct([]);
    for k = 1:Joints.NPoint %Acces the NPoint field of the Joints Struct
        i = Joints.Point(k).Body;
        spi = Joints.Point(k).spi; % spi - point position vector
        sp_earth = BodytoEarth(spi,Bodies(i).p);
        Point_position = Bodies(i).r + sp_earth; %pela expressao do d, r até a origem do referencial sp no ref global ate p
        Data(k).Position = Point_position;
    end
    %Store Vel and Accel Vectors in the fields
    [a,b] = size(vel);
    for j = 1:a
        Data(j).Velocity = vel(j);
        Data(j).Acceleration = acc(j);
    end
    %% Stores said data struct in a cell array per number of iteration
    Points{1,it} = Data;
    it = it+1;
end