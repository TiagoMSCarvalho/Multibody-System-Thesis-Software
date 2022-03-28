function [forceel] = Force_TSpring(forcescount,Bodies,TSpring)
%% Initial variable definitions
% Bodies numbers
i = TSpring(forcescount).Body1;
j = TSpring(forcescount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Force Element location relative to each Bodies coordinate system
spi = Impose_Column(TSpring(forcescount).spi);
spj = Impose_Column(TSpring(forcescount).spj);
% Force Element Constant
k = TSpring(forcescount).Constant;
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Joint location in the global/absolute coordinate system
spig = Ai*spi;
spjg = Aj*spj;
% Initial Displacement
idisplacement = TSpring(forcescount).InitialDisplacement;

%% Vector Calculus and formulation
displacement = rj + spjg -ri - spig;
deltax = displacement - idisplacement;
forcei = k*deltax;
forcej = -k*deltax;
% Allocation of the force to the vector
% Body i
i1 = 6*(i-1)+1;
forceel(i1:i1+2,1) = forcei;
% Body j
i2 = 6*(j-1)+1;
forceel(i2:i2+2,1) = forcej;
end