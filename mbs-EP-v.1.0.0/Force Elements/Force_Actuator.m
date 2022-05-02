function [forceel] = Force_Actuator(forcescount,Bodies,Actuator,ForceFunction,time)
%% Initial variable definitions
% Bodies numbers
i = Actuator(forcescount).Body1;
j = Actuator(forcescount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Force Element location relative to each Bodies coordinate system
spi = Impose_Column(Actuator(forcescount).spi);
spj = Impose_Column(Actuator(forcescount).spj);
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Joint location in the global/absolute coordinate system but in relation
% to the CoM (Pre Processing sp - r)
spig = Ai*spi;
spjg = Aj*spj;

%% Vector Calculus and formulation
% Displacement and Delta Calculus
displacement = rj + spjg -ri - spig;
% Force Direction Vector
[~,lun] = unitvector(displacement);
%% Force Function Magnitude Calculus
sym t
dfunc = str2fun(ForceFunction.Actuator(forcescount).Function);
force = dfunc(time);
%Force Vectors
forcei = force*lun;
forcej = -force*lun;
%Moments Created by the Damper
momenti = cross(spi,forcei);
momentj = cross(spj,forcej);
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