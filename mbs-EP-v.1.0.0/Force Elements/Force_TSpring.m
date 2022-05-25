function [forceel2] = Force_TSpring(forcescount,NBodies,Bodies,TSpring,forceel2)
%% Initial variable definitions
%forceel vector
forceel = zeros(6*NBodies,1);
%Bodies numbers
i = TSpring(forcescount).Body1;
j = TSpring(forcescount).Body2;
%Retrieve p1 and p2
p1 = Bodies(i).p;
p2 = Bodies(j).p;
% Force Element Constant
kt = TSpring(forcescount).Constant;
% Torsion Spring Initial 
t0 = TSpring(forcescount).theta0;
% Axis of the Spring
%axi = TSpring(forcescount).si;
%axj = TSpring(forcescount).sj;
axg = TSpring(forcescount).s;
% (e1,e2,e3) sin(phi/2) - no excel define-se o eixo da mola
%% Vector Calculus and formulation - Torsional
% Calculus of each vector displacement (new si and sj)
i1 = 6*(i-1)+1;
i2 = 6*(j-1)+1;
if t == 0
    forceel(i1:i1+5,1) = 0;
    forceel(i2:i2+5,1) = 0;
 
elseif t~=0 %Calculus of the Displacement
    if axg(1) == 1
        ei1 = p1(1);
        ej1 = p2(1);
        phi1 = 2*asin(ei1);
        phi2 = 2*asin(ej1);
        phi = phi1+phi2;
        torque = kt * (phi-t0);
    elseif axg(1) == 1
        ei2 = p1(2);
        ej2 = p2(2);
        phi1 = 2*asin(ei2);
        phi2 = 2*asin(ej2);
        phi = phi1+phi2;
        torque = kt * (phi-t0);
    elseif axg(1) == 1
        ei3 = p1(3);
        ej3 = p2(3);
        phi1 = 2*asin(ei3);
        phi2 = 2*asin(ej3);
        phi = phi1+phi2;
        torque = kt * (phi-t0);
    end
    forceel(i1:i1+2,1) = 0;
    forceel(i1+3:i1+5,1) = torque;
    forceel(i2:i2+2,1) = 0;
    forceel(i2+3:i2+5,1) = -torque;    
end
%Add to the existing vector
forceel2 = forceel2 + forceel;
end