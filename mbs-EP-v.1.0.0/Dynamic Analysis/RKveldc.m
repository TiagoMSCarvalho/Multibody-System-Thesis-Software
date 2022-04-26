function [phid] = RKveldc(Joints,Bodies,Flags,driverfunctions,t)
%Calculates the Jacobian for the velocity direct correction.
%Direct Correction is now seperated due to the fact that the position
%correction needs to be solved iteratively.
Jacobian = [];
funCount = 1;
% For the Ground Constraints
for jointCount=1:Joints.NGround
    [~,Jacobian,~,~,funCount] = Ground_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Ground,Flags);
end
% For the Spherical joints
for jointCount=1:Joints.NSpherical
    [~,Jacobian,~,~,funCount] = Joint_Spherical([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
end
% For the Composite Spherical Joint (SPH - SPH)
for jointCount=1:Joints.NCompSpherical
    [~,Jacobian,~,~,funCount] = Joint_CompSpherical([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
end
% For the Universal joints
for jointCount=1:Joints.NUniversal
    [~,Jacobian,~,~,funCount] = Joint_Universal([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Universal,Flags); 
end
% For the Revolute joints
for jointCount=1:Joints.NRevolute
    [~,Jacobian,~,~,funCount] = Joint_Revolute([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Revolute,Flags);
end
% For the Cylindrical joints
for jointCount=1:Joints.NCylindrical
    [~,Jacobian,~,~,funCount] = Joint_Cylindrical([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
end 
% For the Translation joints
for jointCount=1:Joints.NTranslation
    [~,Jacobian,~,~,funCount] = Joint_Translation([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Translation,Flags);
end
% For the Translation joints
for jointCount=1:Joints.NSphRev
    [~,Jacobian,~,~,funCount] = Joint_CompSphRev([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
end
% For the Translation Revolute Composite joint
for jointCount=1:Joints.NTraRev
    [~,Jacobian,~,~,funCount] = Joint_CompTraRev([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
end 
% For the Simple Constraints
for jointCount=1:Joints.NSimple
    [~,Jacobian,~,~,funCount] = Simple_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
end
% For the Driver Constraints
for jointCount=1:Joints.NDriver
    [~,Jacobian,~,~,funCount] = Driver_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,t,driverfunctions);
end

phid = Jacobian; %variable allocation
end