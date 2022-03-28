function [forceel] = Force_Damper(forcescount,Bodies,Damper)
%% Initial variable definitions
% Bodies numbers
i = Damper(forcescount).Body1;
j = Damper(forcescount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Force Element location relative to each Bodies coordinate system
spi = Impose_Column(Damper(forcescount).spi);
spj = Impose_Column(Damper(forcescount).spj);
% Force Element Constant - Damper Coeff
c = Damper(forcescount).Constant;
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Joint location in the global/absolute coordinate system
spig = Ai*spi;
spjg = Aj*spj;
% Initial Displacement
idisplacement = Damper(forcescount).InitialDisplacement;
%% Displacement Derivative
% Translational Velocities
rdi = Bodies(i).rd;
rdj = Bodies(j).rd;
% Angular Velocities
wi = Bodies(i).w;
wj = Bodies(j).w;
% Angular Velocities Skew Matrices
swi = SkewMatrix3(wi);
swj = SkewMatrix3(wj);
% Derivatives of the global spi and spj
spid = Ai*swi*spi;
spjd = Aj*swj*spj;

%% Vector Calculus and formulation
% Displacement and Delta Calculus
displacement = rj + spjg -ri - spig;
deltax = displacement - idisplacement;
% Displacement derivative
ddisp = rdj + spjd - rdi - spid;
% Force Direction Vector
[lmag,lun] = unitvector(displacement);
lengthrateofchange = (1/lmag)*(displacement'*ddisp);
% Force Magnitudes
force = k*lengthrateofchange;
%Force Vectors
forcei = force*lun;
forcej = -force*lun;
%Moments Created by the Damper
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
forceel(i2+3:i2+5,1) = forcei;
end