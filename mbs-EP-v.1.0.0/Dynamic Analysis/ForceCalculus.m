 function [vectorg] = ForceCalculus(Forces,NBodies,Bodies,Grav)
%Assemblies the Vector g to enable the calculation of the initial
%acceleration
%Pre Allocation of the Vectors
vectorg = zeros(6*NBodies,1);
forceel = zeros(6*NBodies,1);
%Allocation of the gravity properties and vector
gdir = string(Grav.Direction);
gmag = cell2mat(Grav.Magnitude);
if strcmp(gdir,"x") == 1
    g = [gmag;0;0];
elseif strcmp(gdir,"y") == 1
    g = [0;gmag;0];
elseif strcmp(gdir,"z") == 1
    g = [0;0;gmag];
end

%Calculus of the Force Elements
for forcescount = 1:Forces.NSpring
    [forceel] = Force_Spring(forcescount,Bodies,Forces.Spring);
end
for forcescount = 1:Forces.NTSpring
    [forceel] = Force_TSpring(forcescount,Bodies,Forces.TSpring);
end
for forcescount = 1:Forces.NDamper
    [forceel] = Force_Damper(forcescount,Bodies,Forces.Damper);
end
%Allocation of the forces to the bodies and calculus of: swJw and weight
for i = 1:NBodies
    Mass = Bodies(i).Mass;
    Inertia = Bodies(i).Inertia; %Tem de ser posto em matriz
    A = Bodies(i).A;
    I = diag(Inertia);
    w = Bodies(i).w;
    sw = SkewMatrix3(w);
    %Ia = A*I*A'; Alterar para testar J'
    wJw = sw*I*w;
    i1 = 6*(i-1)+1;
    if isnan(gmag) == 1
    vectorg(i1:i1+2,1) = Impose_Column(Bodies(i).Force);
    vectorg(i1+3:i1+5,1) = Impose_Column(Bodies(i).Torque) - Impose_Column(wJw);        
    elseif isnan(gmag) ~= 1
    vectorg(i1:i1+2,1) = Impose_Column(Bodies(i).Force) + (Mass*eye(3))*g;
    vectorg(i1+3:i1+5,1) = Impose_Column(Bodies(i).Torque) - Impose_Column(wJw);
    end
end

vectorg = vectorg + forceel;

end

