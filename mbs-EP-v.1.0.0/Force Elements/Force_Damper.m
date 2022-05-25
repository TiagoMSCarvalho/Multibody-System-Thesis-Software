function [forceel3] = Force_Damper(forcescount,NBodies,Bodies,Damper,ForceFunction,forceel3)
%% Initial variable definitions
%forceel
forceel = zeros(6*NBodies,1);
% Bodies numbers
i = Damper(forcescount).Body1;
j = Damper(forcescount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Force Element location relative to each Bodies coordinate system
spi = Impose_Column(Damper(forcescount).spi);
spj = Impose_Column(Damper(forcescount).spj);
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Joint location in the global/absolute coordinate system but in relation
% to the CoM (Pre Processing sp - r)
spig = Ai*spi;
spjg = Aj*spj;
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
%Skew spi and spj
sspi = SkewMatrix3(spig);
sspj = SkewMatrix3(spjg);

%% Vector Calculus and formulation
% Displacement and Delta Calculus
displacement = rj + spjg - ri - spig;    
% Displacement derivative
ddisp = rdj + spjd - rdi - spid;
% Force Direction Vector
[lmag,lun] = unitvector(displacement);
lengthrateofchange = (1/lmag)*(displacement'*ddisp);
% Force Magnitude Calculus
if ~isnan(Damper(forcescount).Constant)
    % Linear Spring
    c = Damper(forcescount).Constant; %damping coefficient
    force = c*lengthrateofchange;
elseif isnan(Damper(forcescount).Constant)
    % Non Linear Spring:
        %Function Input: Velocity of compression ( lengthrateofchange)
        %Function Output: Damper Force
    sym x
    Noffun = Damper(forcescount).Noffun;
    if Noffun == 1
        dfunc = str2func(Damper(forcescount).Function1);
        force = dfunc(lengthrateofchange);
    elseif Noffun == 2
        if lengthrateofchange <= Damper(forcescount).Intmin
            dfunc = str2func(ForceFunction.Damper(forcescount).Function1);
            force = dfunc(lengthrateofchange);
        elseif lenthrateofchange > Damper(forcescount).Intmin
            dfunc = str2func(ForceFunction.Damper(forcescount).Function2);
            force = dfunc(lenthrateofchange);
        end
    elseif Noffun == 3
        if lengthrateofchange <= Damper(forcescount).Intmin
            dfunc = str2fun(ForceFunction.Damper(forcescount).Function1);
            force = dfunc(lengthrateofchange);
        elseif lengthrateofchange < Damper(forcescount).Intmax && lengthrateofchange > Damper(forcescount).Intmin
            dfunc = str2fun(ForceFunction.Damper(forcescount).Function2);
            force = dfunc(lengthrateofchange);
        elseif lengthrateofchange >= Damper(forcescount).Intmax
            dfunc = str2fun(ForceFunction.Damper(forcescount).Function3);
            force = dfunc(lengthrateofchange);
        end
    end
end
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
%Add to the existing vector
forceel3 = forceel3 + forceel;
end