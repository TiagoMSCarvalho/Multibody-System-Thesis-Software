function [forceel4] = Force_Actuator(forcescount,NBodies,Bodies,Actuator,ForceFunction,time,forceel4,coord)
%% Initial variable definitions
%forceel
if coord == 7
    forceel = zeros(7*NBodies,1);
elseif coord == 6
    forceel = zeros(6*NBodies,1);
end
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
% Skew Matrix
sspi = SkewMatrix3(spi);

%% Vector Calculus and formulation
% Displacement and Delta Calculus
displacement = rj + spjg -ri - spig;
% Force Direction Vector
[~,lun] = unitvector(displacement);
%% Force Function Magnitude Calculus
Noffun = Actuator(forcescount).Noffun;
if Noffun == 1
    dfunc = str2func(ForceFunction.Actuator(forcescount).Function1);
    force = dfunc(time);
elseif Noffun == 2
    if time <= Actuator(forcescount).Intmin
        dfunc = str2func(ForceFunction.Actuator(forcescount).Function1);
        force = dfunc(time);
    elseif time > Actuator(forcescount).Intmax
        dfunc = str2func(ForceFunction.Actuator(forcescount).Function2);
        force = dfunc(time);
    end
elseif Noffun == 3
    if time <= Actuator(forcescount).Intmin
        dfunc = str2func(ForceFunction.Actuator(forcescount).Function1);
        force = dfunc(time);
    elseif time < Actuator(forcescount).Intmax && time > Actuator(forcescount).Intmin
        dfunc = str2func(ForceFunction.Actuator(forcescount).Function2);
        force = dfunc(time);
    elseif time >= Actuator(forcecount).Intmax
        dfunc = str2func(ForceFunction.Actuator(forcescount).Function3);
        force = dfunc(time);
    end
end
%Force Vectors
forcei = force*lun;
forcej = -force*lun;
%Moments Created by the Actuator
momenti = sspi*Ai'*forcei;
momentj = sspj*Aj'*forcej;
%Allocation of the force to the vector
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
    i1 = 6*(i-1)+1;
    forceel(i1:i1+2,1) = forcei;
    forceel(i1+3:i1+5,1) = momenti;
    % Body j
    i2 = 6*(j-1)+1;
    forceel(i2:i2+2,1) = forcej;
    forceel(i2+3:i2+5,1) = momentj; 
end
%Add to the existing vector
forceel4 = forceel4 + forceel; 
end