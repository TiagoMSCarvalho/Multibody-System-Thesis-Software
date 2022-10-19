function [forceel2] = Force_TSpring(forcescount,NBodies,Bodies,TSpring,forceel2,coord)
%% Initial variable definitions
%forceel vector
if coord == 7
    forceel = zeros(7*NBodies,1);
elseif coord == 6
    forceel = zeros(6*NBodies,1);
elseif coord == 8
    forceel = zeros(8*NBodies,1);
end
%Bodies numbers
i = TSpring(forcescount).Body1;
j = TSpring(forcescount).Body2;
%Transform
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Force Element Constant
kt = TSpring(forcescount).Constant;
% Torsion Spring Initial 
t0 = TSpring(forcescount).theta0;
% Vectors Perpendicular to the Axis
sig = Ai*TSpring(forcescount).si;
sjg = Aj*TSpring(forcescount).sj;
nsig = norm(sig);
nsjg = norm(sjg);
% Axis vector and ui and uj
axisg = TSpring(forcescount).s;
ui = TSpring(forcescount).ui;
uj = TSpring(forcescount).uj;

ssig = SkewMatrix3(sig); % skew matrix si
% (e1,e2,e3) sin(phi/2) - no excel define-se o eixo da mola
%% Vector Calculus and formulation - Torsional
% Calculus of each vector displacement (new si and sj)
if coord == 7
    i1 = 7*(i-1)+1;
    i2 = 7*(j-1)+1;
elseif coord == 6
    i1 = 6*(i-1)+1;
    i2 = 6*(j-1)+1;
end

if t == 0
    if coord == 7
        forceel(i1:i1+6,1) = 0;
        forceel(i2:i2+6,1) = 0;
    elseif coord == 6
        forceel(i1:i1+5,1) = 0;
        forceel(i2:i2+5,1) = 0;
    end
elseif t~=0 %Correction Using the EP for 3D from Nikravesh
    phi = acos((sig'*sjg)/(dot(nsig,nsjg)));
    signal = axisg'*ssig*sjg;
    if signal < 0
        phi = phi + pi;
    end
        
    if ~isnan(TSpring(forcescount).Constant)
        torque = kt*(phi-t0)*u;
        torquei = torque*ui;
        torquej = torque*uj;
    elseif isnan(TSpring(forcescount).Constant)
        deltaphi = phi-t0;
        Noffun = TSpring(forcescount).Noffun;
        if Noffun == 1
            dfunc = str2func(ForceFunction.TSpring(forcescount).Function1);
            torque = dfunc(deltaphi);
        elseif Noffun == 2
            if deltaphi <= TSpring(forcescount).Intmin
                dfunc = str2func(ForceFunction.TSpring(forcescount).Function1);
                torque = dfunc(deltaphi);
            elseif deltaphi > TSpring(forcescount).Intmax
                dfunc = str2func(ForceFunction.TSpring(forcescount).Function2);
                torque = dfunc(deltaphi);
            end
        elseif Noffun == 3
            if deltaphi <= TSpring(forcescount).Intmin
                dfunc = str2func(ForceFunction.TSpring(forcescount).Function1);
                torque = dfunc(deltaphi);
            elseif deltaphi < TSpring(forcescount).Intmax && deltaphi > TSpring(forcescount).Intmin
                dfunc = str2func(ForceFunction.TSpring(forcescount).Function2);
                torque = dfunc(deltaphi);
            elseif deltaphi >= TSpring(forcescount).Intmax
                dfunc = str2func(ForceFunction.TSpring(forcescount).Function3);
                torque = dfunc(deltaphi);
            end
        end
    end                 
    torquei = torque*ui;
    torquej = torque*uj;
    if coord == 7
        forceel(i1:i1+2,1) = 0;
        forceel(i1+3:i1+6,1) = 2*Bodies(i).L'*torquei;
        forceel(i2:i2+2,1) = 0;
        forceel(i2+3:i2+6,1) = -2*Bodies(j).L'*torquej;    
    elseif coord == 6
        forceel(i1:i1+2,1) = 0;
        forceel(i1+3:i1+5,1) = torquei;
        forceel(i2:i2+2,1) = 0;
        forceel(i2+3:i2+5,1) = -torquej; 
    elseif coord == 8
        i1 = 8*(i-1)+1;
        i2 = 8*(j-1)+1;
        forceel(i1:i1+2,1) = 0;
        forceel(i1+3:i1+6,1) = 2*Bodies(i).L'*torquei;
        forceel(i2:i2+2,1) = 0;
        forceel(i2+3:i2+6,1) = -2*Bodies(j).L'*torquej;    
    end
end
%Add to the existing vector
forceel2 = forceel2 + forceel;
end