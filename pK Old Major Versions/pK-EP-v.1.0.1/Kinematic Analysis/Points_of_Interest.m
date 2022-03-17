function [Points] = Points_of_Interest(nIteration,Points,Bodies,Joints)
for k = 1:Joints.NPoint %Acces the NPoint field of the Joints Struct
    i = Joints.Point(k).Body;
    spi = Joints.Point(k).spi; % spi - point position vector
    sp_earth = BodytoEarth(spi,Bodies(i).p);
    Point_position = Bodies(i).r + sp_earth; %pela expressao do d, r até a origem do referencial sp no ref global ate p
    Points{k,nIteration} = Point_position; % {k,1} goes into the cell array
    %It now stores the vector Point_Position in each cell, this is done to
    %ease the debug process.
end
end