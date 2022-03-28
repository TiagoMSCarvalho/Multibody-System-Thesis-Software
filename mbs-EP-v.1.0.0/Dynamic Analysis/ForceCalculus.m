function [vetorg] = ForceCalculus(Forces,NBodies,Bodies)
%Assemblies the Vector g to enable the calculation of the initial
%acceleration

vetorg = zeros(6*NBodies,1);
forceel = zeros(6*NBodies,1);
funCount = 1;

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
%Allocation of the forces to the bodies
for i = 1:NBodies
    
    
end



end

