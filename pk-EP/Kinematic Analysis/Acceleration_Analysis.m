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


function Bodies = Acceleration_Analysis(Jacobian,Joints,NBodies,Bodies)
% Function that controls the acceleration analysis

%Form the velocity contraint equation
Flags.Position = 0;
Flags.Jacobian = 1;
Flags.Velocity = 1;
Flags.Acceleration = 1;
funCount = 1;
gamma=[];
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
% For the Ground Constraints
for jointCount=1:Joints.NGround
    [~,~,~,gamma,funCount] = Ground_Constraints([],[],[],gamma,funCount,jointCount, Bodies, Joints.Ground,Flags);
end
% For the Simple Constraints
for jointCount=1:Joints.NSimple
    [~,~,~,gamma,funCount] = Simple_Constraints([],[],[],gamma,funCount,jointCount, Bodies, Joints.Simple,Flags);
end
% For the Euler Parameter Constraint
for NBod = 1:NBodies
    [~,~,~,gamma,funCount] = EulerParameterConstraint([],[],[],gamma,funCount,NBod,Bodies,Flags);
end
% For the Driver Constraints
for jointCount=1:Joints.NDriver
    [~,~,~,gamma,funCount] = Driver_Constraints([],[],[],gamma,funCount,jointCount, Bodies, Joints.Driver,Flags,0);
end
% Solve the system of (acceleration) equations
qdd = Jacobian\Impose_Column(gamma);

% Store the accelerations in proper variables
Bodies = UpdateAccelerations(qdd,NBodies,Bodies);

%Finish function
end