function [forceel] = Force_Spring(forcescount,Bodies,Spring)
%% Initial variable definitions
% Bodies numbers
i = Spring(forcescount).Body1;
j = Spring(forcescount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Force Element location relative to each Bodies coordinate system
spi = Impose_Column(Spring(forcescount).spi);
spj = Impose_Column(Spring(forcescount).spj);
% Force Element Constant - Stiffness Constant
k = Spring(forcescount).Constant;
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Joint location in the global/absolute coordinate system
spig = Ai*spi;
spjg = Aj*spj;
% Initial Displacement
idisplacement = Spring(forcescount).InitialDisplacement;

%% Vector Calculus and formulation
displacement = rj + spjg -ri - spig;
deltax = displacement - idisplacement;
%Force Magnitudes
force = k*deltax;
%Force Direction Vector
[~,lun] = unitvector(displacement);
%Force Vectors
forcei = force*lun;
forcej = -force*lun;
%Moments Created by the Translational Spring
momenti = cross(spig,forcei);
momentj = cross(spjg,forcej);
% Allocation of the force to the vector
% Body i
i1 = 6*(i-1)+1;
forceel(i1:i1+2,1) = forcei;
forceel(i1+3:i1+5,1) = momenti;
% Body j
i2 = 6*(j-1)+1;
forceel(i2:i2+2,1) = forcej;
forceel(i2+3:i2+5,1) = momentj;
end