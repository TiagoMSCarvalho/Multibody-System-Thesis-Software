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

function [Bodies,Jacobian,vel,qd] = Velocity_Analysis(Joints,NBodies,Bodies,debugdata,ang,t,driverfunctions,SimType)
%Function that controls the Kinematic velocity analysis

%Form the velocity constraint equation
Flags.Position = 0;
Flags.Jacobian = 1;
Flags.Velocity = 1;
Flags.Acceleration = 0;
Flags.Dynamic = 0;
Flags.AccelDyn = 0;

funCount = 1;
Jacobian = zeros(debugdata(1).cdof,NBodies*7);
niu = zeros(debugdata(1).cdof,1);
Ct = zeros(debugdata(1).cdof,1);
vel = [];
%% Determining the Jacobian of the Multibody problem 
% For the Ground Constraints
for jointCount=1:Joints.NGround
    [~,Jacobian,niu,~,funCount] = Ground_Constraints([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Ground,Flags);
% For the Spherical joints
for jointCount=1:Joints.NSpherical
    [~,Jacobian,niu,~,funCount] = Joint_Spherical([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
end
% For the Composite Spherical Joint (SPH - SPH)
for jointCount=1:Joints.NCompSpherical
    [~,Jacobian,niu,~,funCount] = Joint_CompSpherical([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
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
% For the Spherical Revolute joints
for jointCount=1:Joints.NSphRev
    [~,Jacobian,niu,~,funCount] = Joint_CompSphRev([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
end 
% For the Translation Revolute Composite joints
for jointCount=1:Joints.NTraRev
    [~,Jacobian,niu,~,funCount] = Joint_CompTraRev([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
end 
% For the Simple Constraints
for jointCount=1:Joints.NSimple
    [~,Jacobian,niu,~,funCount] = Simple_Constraints([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Simple,Flags);
end
% For the Euler Parameter Constraint
for NBod = 2:(NBodies) %takes the first body, ground out of the equation
    [~,Jacobian,niu,~,funCount] = EulerParameterConstraint([],Jacobian,niu,[],funCount,NBod,Bodies,Flags);
end
% For the Driver Constraints
for jointCount=1:Joints.NDriver
    [~,Jacobian,Ct,~,funCount] = Driver_Constraints([],Jacobian,Ct,[],funCount,jointCount, Bodies, Joints.Driver,Flags,t,driverfunctions);
end

%% Linear System Solver: Implementation of Robust IK - Least Squares
condv = cond(Jacobian);
niuc = Impose_Column(niu);
Ct = Impose_Column(Ct);
qd = Jacobian\(niuc-Ct);

%% Post Data treatment for the rotational parameters


for i = 1:NBodies
    i1 = 7*(i-1)+1;
    i2 = 6*(i-1)+1;
    p = Bodies(i).p;
    Gi = Bodies(i).G;
    Ei = [p(1),p(2),p(3),p(4);Gi]; %From J.Haug Book
    pd = qd(i1+3:i1+6);
    vel(i2:i2+2,1) = qd(i1:i1+2);
    w = 2*Ei*pd;
    vel(i2+3:i2+5,1) = w(2:4);    
end

% Store the velocities in proper variables
Bodies = UpdateVelocities(qd,NBodies,Bodies,SimType);

end