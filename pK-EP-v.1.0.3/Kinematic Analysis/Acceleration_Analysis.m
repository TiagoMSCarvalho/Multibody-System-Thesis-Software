%% Acceleration Analysis Function Details
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
%    every body plus the already obtained Jacobian matrix, the right hand
%    side of the acceleration equations, gamma, is calculated to solve the
%    equations


function [Bodies,acc] = Acceleration_Analysis(Jacobian,Joints,NBodies,Bodies,debugdata,ang,t)
% Function that controls the acceleration analysis

%Form the velocity contraint equation
Flags.Position = 0;
Flags.Jacobian = 0;
Flags.Velocity = 0;
Flags.Acceleration = 1;
funCount = 1;
gamma = zeros(debugdata(1).cdof,1);
Ctt = zeros(debugdata(1).cdof,1);
acc = [];
%% Determining the Gamma of the Multibody problem
% For the Ground Constraints
for jointCount=1:Joints.NGround
    [~,~,~,gamma,funCount] = Ground_Constraints([],[],[],gamma,funCount,jointCount, Bodies, Joints.Ground,Flags);
% For the Spherical joints
for jointCount=1:Joints.NSpherical
    [~,~,~,gamma,funCount] = Joint_Spherical([],[],[],gamma,funCount,jointCount, Bodies, Joints.Spherical,Flags);
end
% For the Universal joints
for jointCount=1:Joints.NUniversal
    [~,~,~,gamma,funCount] = Joint_Universal([],[],[],gamma,funCount,jointCount, Bodies, Joints.Universal,Flags); 
end
% For the Revolute joints
for jointCount=1:Joints.NRevolute
    [~,~,~,gamma,funCount] = Joint_Revolute([],[],[],gamma,funCount,jointCount, Bodies, Joints.Revolute,Flags);
end
% For the Cylindrical joints
for jointCount=1:Joints.NCylindrical
    [~,~,~,gamma,funCount] = Joint_Cylindrical([],[],[],gamma,funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
end 
% For the Translation joints
for jointCount=1:Joints.NTranslation
    [~,~,~,gamma,funCount] = Joint_Translation([],[],[],gamma,funCount,jointCount, Bodies, Joints.Translation,Flags);
end 
% For the Simple Constraints
for jointCount=1:Joints.NSimple
    [~,~,~,gamma,funCount] = Simple_Constraints([],[],[],gamma,funCount,jointCount, Bodies, Joints.Simple,Flags);
end
% For the Euler Parameter Constraint
for NBod = 2:(NBodies) %takes the first body, ground out of the equation
    [~,~,~,gamma,funCount] = EulerParameterConstraint([],[],[],gamma,funCount,NBod,Bodies,Flags);
end
% For the Driver Constraints
for jointCount=1:Joints.NDriver
    [~,~,~,Ctt,funCount] = Driver_Constraints([],[],[],Ctt,funCount,jointCount, Bodies, Joints.Driver,Flags,t,ang);
end

%% Linear System Solver: Implementation of Robust IK - Least Squares
conda = cond(Jacobian);
gammac = Impose_Column(gamma);
Ctt = Impose_Column(Ctt);

if conda <= 10^1
qdd = Jacobian\(gammac-Ctt);
elseif conda > 10^1
qdd = Jacobian\(gammac-Ctt);
%Possible algorithms: pcg,equilibrate,lsqr;
end
%% Post Data treatment for the rotational parameters


for i = 1:NBodies
    i1 = 7*(i-1)+1;
    i2 = 6*(i-1)+1;
    acc(i2:i2+2,1) = qdd(i1:i1+2);
    Gi = Bodies(i).G;
    pdd = qdd(i1+3:i1+6);
    acel = 2*Gi*pdd;
    acc(i2+3:i2+5,1) = acel;    
end

% Store the accelerations in proper variables
Bodies = UpdateAccelerations(qdd,NBodies,Bodies);

end