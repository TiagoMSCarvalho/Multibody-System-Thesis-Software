%% Velocity Analysis Function Details
%
% Inputs and Outputs:
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - Joints: structure with all the information relative to the joints
%    and constraints divided in fields correspondent to each type of
%    joint/constraint
%    - Jacobian: the Jacobian matrix needed to compute velocity and acceleration equations 
%
% Objective of the Function:
%    Starting with the already calculated positions and orientations of
%    every body, the Jacobian is calculated to solve the velocity
%    equations. The right hand side of this equations, niu, is also
%    calculated. In the end the velocities of the bodies are updated in the
%    Bodies structure

function [Bodies,Jacobian,qd] = Velocity_Analysis(Joints,NBodies,Bodies,debugdata)

%Form the velocity constraint equation
Flags.Position = 0;
Flags.Jacobian = 1;
Flags.Velocity = 1;
Flags.Acceleration = 0;
funCount = 1;
Jacobian=zeros(debugdata(1).cdof,NBodies*7);
niu=[];
% For the Spherical joints
for jointCount=1:Joints.NSpherical
    [~,Jacobian,niu,~,funCount] = Joint_Spherical([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
end

% For the Universal joints
for jointCount=1:Joints.NUniversal
    [~,Jacobian,niu,~,funCount] = Joint_Universal([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Universal,Flags); 
end
% For the Revolute joints
for jointCount=1:Joints.NRevolute
    [~,Jacobian,niu,~,funCount] = Joint_Revolute([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Revolute,Flags);
end
% For the Cylindrical joints
for jointCount=1:Joints.NCylindrical
    [~,Jacobian,niu,~,funCount] = Joint_Cylindrical([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
end 
% For the Translation joints
for jointCount=1:Joints.NTranslation
    [~,Jacobian,niu,~,funCount] = Joint_Translation([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Translation,Flags);
end 
% For the Ground Constraints
for jointCount=1:Joints.NGround
    [~,Jacobian,niu,~,funCount] = Ground_Constraints([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Ground,Flags);
end
% For the Simple Constraints
for jointCount=1:Joints.NSimple
    [~,Jacobian,niu,~,funCount] = Simple_Constraints([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Simple,Flags);
end
% For the Euler Parameter Constraint
for NBod = 1:NBodies
    [~,Jacobian,niu,~,funCount] = EulerParameterConstraint([],Jacobian,niu,[],funCount,NBod,Bodies,Flags);
end
% For the Driver Constraints
for jointCount=1:Joints.NDriver
    [~,Jacobian,niu,~,funCount] = Driver_Constraints([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Driver,Flags,0);
end

%Implementation of Robust IK - Least Squares

condv = cond(Jacobian);
niuc = Impose_Column(niu);

if condv <= 10^1
qd = Jacobian\niuc;
elseif condv > 10^1
qd = lsqr(Jacobian,niuc);
end
% Store the velocities in proper variables
Bodies = UpdateVelocities(qd,NBodies,Bodies);

end