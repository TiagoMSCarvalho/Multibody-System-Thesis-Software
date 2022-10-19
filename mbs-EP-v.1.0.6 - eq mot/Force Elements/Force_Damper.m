function [forceel3] = Force_Damper(forcescount,NBodies,Bodies,Damper,ForceFunction,forceel3,coord)
%% Initial variable definitions
%forceel
if coord == 7
    forceel = zeros(7*NBodies,1);
elseif coord == 6
    forceel = zeros(6*NBodies,1);
elseif coord == 8
    forceel = zeros(8*NBodies,1);
end
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
spid = swi*Ai*spi;
spjd = swj*Aj*spj;
%Skew spi and spj
sspi = SkewMatrix3(spi);
sspj = SkewMatrix3(spj);

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
    % Sym x was deleted the functions are now anonymous and work as a
    % function of F(dx), where dx is the velocity of compression or tension
    % of the damper.
    Noffun = Damper(forcescount).Noffun;
    if Noffun == 1
        dfunc = str2func(Damper(forcescount).Function1);
        force = dfunc(lengthrateofchange);
    elseif Noffun == 2
        if lengthrateofchange <= Damper(forcescount).Intmin
            dfunc = str2func(ForceFunction.Damper(forcescount).Function1);
            force = dfunc(lengthrateofchange);
        elseif lenthrateofchange > Damper(forcescount).Intmax
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
momenti = sspi*Ai'*forcei;
momentj = sspj*Aj'*forcej;
% Allocation of the force to the vector
if coord == 7
    % Body i
    i1 = 7*(i-1)+1;
    forceel(i1:i1+2,1) = forcei;
    forceel(i1+3:i1+6,1) = 2*Bodies(i).L'*momenti;
    % Body j
    i2 = 7*(j-1)+1;
    forceel(i2:i2+2,1) = forcej;
    forceel(i2+3:i2+6,1) = 2*Bodies(j).L'*momentj;
elseif coord == 6
    % Body i
    i1 = 6*(i-1)+1;
    forceel(i1:i1+2,1) = forcei;
    forceel(i1+3:i1+5,1) = momenti;
    % Body j
    i2 = 6*(j-1)+1;
    forceel(i2:i2+2,1) = forcej;
    forceel(i2+3:i2+5,1) = momentj;
elseif coord == 8
    % Body i
    i1 = 8*(i-1)+1;
    forceel(i1:i1+2,1) = forcei;
    forceel(i1+3:i1+6,1) = 2*Bodies(i).L'*momenti;
    % Body j
    i2 = 8*(j-1)+1;
    forceel(i2:i2+2,1) = forcej;
    forceel(i2+3:i2+6,1) = 2*Bodies(j).L'*momentj;
end
%Add to the existing vector
forceel3 = forceel3 + forceel;
end