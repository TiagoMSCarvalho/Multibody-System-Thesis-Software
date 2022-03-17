function Points = Points_of_Interest(j,Points,Bodies,Joints)
for k=1:Joints.NPoint %Acces the NPoint field of the Joints Struct
    i = Joints.Point(k).Body;
    spi = Joints.Point(k).spi; % spi - point position vector
    sp_earth = BodytoEarth(spi,Bodies(i).p);
    Point_position = Bodies(i).r + sp_earth;  %pela expressao do d, r até a origem do referencial sp no ref global ate p
    Points{k,1}(j) = Point_position(1); % {k,1} goes into the cell array
    Points{k,2}(j) = Point_position(2); % (1) taps in to the vector
    Points{k,3}(j) = Point_position(3);
end
end