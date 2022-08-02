function [forceel1] = Force_Spring(forcescount,NBodies,Bodies,Spring,ForceFunction,forceel1,coord)
%% Initial variable definitions
%forceel vector
if coord == 7
    forceel = zeros(7*NBodies,1);
elseif coord == 6
    forceel = zeros(6*NBodies,1);
end
% Bodies numbers
i = Spring(forcescount).Body1;
j = Spring(forcescount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Force Element location relative to each Bodies coordinate system
spi = Impose_Column(Spring(forcescount).spi);
spj = Impose_Column(Spring(forcescount).spj);
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Joint location in the global/absolute coordinate system
spig = Ai*spi;
spjg = Aj*spj;
% Initial Displacement
idisplacement = Spring(forcescount).InitialDisplacement;
% Null Force Length if exists
nl = Spring(forcescount).NullLength;
% Force Direction Vector = Initial Direction Vector
[~,lun] = unitvector(idisplacement);
%Skew Matrix
sspi = SkewMatrix3(spi);
sspj = SkewMatrix3(spj);


%% Vector Calculus and formulation
displacement = rj + spjg -ri - spig;
[magd,~] = unitvector(displacement);
if isnan(nl)
    [magid,~] = unitvector(idisplacement);
    deltaxmag = magd - magid;
elseif ~isnan(nl)
    deltaxmag = magd - nl;
end
% Force Magnitude Calculus
if ~isnan(Spring(forcescount).Constant)
    % Linear Spring
    k = Spring(forcescount).Constant; %Stiffness Constant
    force = k*deltaxmag; %Hooke Law
elseif isnan(Spring(forcescount).Constant)
    %Non Linear Spring
        %Function Input: Displacement of the Spring
        %Function Output: Spring Force
%     sym dx
%     sfunc = str2func(ForceFunction.Spring(forcescount).Function);
%     force = sfunc(displacement);
    sym dx
    Noffun = Spring.Noffun;
    if Noffun == 1
        dfunc = str2func(ForceFunction.Spring(forcescount).Function1);
        force = dfunc(deltaxmag);
    elseif Noffun == 2
        if lengthrateofchange <= Spring(forcescount).Intmin
            dfunc = str2func(ForceFunction.Spring(forcescount).Function1);
            force = dfunc(deltaxmag);
        elseif lenthrateofchange > Spring(forcescount).Intmin
            dfunc = str2func(ForceFunction.Spring(forcescount).Function2);
            force = dfunc(deltaxmag);
        end
    elseif Noffun == 3
        if lengthrateofchange <= Spring(forcescount).Intmin
            dfunc = str2fun(ForceFunction.Spring(forcescount).Function1);
            force = dfunc(deltaxmag);
        elseif lengthrateofchange < Spring(forcescount).Intmax && lengthrateofchange > Spring(forcescount).Intmin
            dfunc = str2fun(ForceFunction.Spring(forcescount).Function2);
            force = dfunc(deltaxmag);
        elseif lengthrateofchange >= Spring(forcescount).Intmax
            dfunc = str2fun(ForceFunction.Spring(forcescount).Function3);
            force = dfunc(deltaxmag);
        end
    end
end
%Force Vectors
forcei = force*lun;
forcej = force*lun;
%Moments Created by the Translational Spring
momenti = cross(spi,forcei);
momentj = cross(spj,forcej);
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
    forceel(i1+3:i1+5,1) = sspi*Ai*forcei;
    % Body j
    i2 = 6*(j-1)+1;
    forceel(i2:i2+2,1) = forcej;
    forceel(i2+3:i2+5,1) = sspj*Aj*forcej;
end
%Add to the existing vector
forceel1 = forceel1 + forceel;
end