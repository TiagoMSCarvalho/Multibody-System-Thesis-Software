function [DynAcc,LagMulti,Jacobian,Bodies] = DynInitialAccel(NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,time,driverfunctions,debugdata,ForceFunction)
%This function uses the inputs of initial position, initial velocities and
%forces to calculate the initial acceleration that will be fed to the
%Runge-Kutta ODE45 solver.

%% Set Up of the variables needed to construct the Matrix - Dynamic Modified Jacobian
% Pre Allocation of the q0 vector, for the calc of AGL
    q0 = CreateAuxiliaryBodyStructure(NBodies,Bodies);
% Calculation of the Matrixes A, G and L
    Bodies = DynCalcAGL(q0,NBodies,Bodies);
% Allocation of the Flags Struct
    % Flags.Dynamic, is a new field that was introduced to help to separate
    % the Kinematic Program from the Dynamic Program, and to keep the
    % program modular and of easy debug.
    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 1;
    Flags.AccelDyn = 1;
    funCount = 1;
    Jacobian = [];
    gamma = zeros(debugdata(1).cdof - NBodies,1);
    Ctt = zeros(debugdata(1).cdof - NBodies,1);
% Assembly of the Jacobian and rhs acceleration vector.
    % For the Ground Constraints
    for jointCount=1:Joints.NGround
        [~,Jacobian,~,gamma,funCount] = Ground_Constraints([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Ground,Flags);
    end
    % For the Spherical joints
    for jointCount=1:Joints.NSpherical
        [~,Jacobian,~,gamma,funCount] = Joint_Spherical([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Spherical,Flags);
    end
    % For the Composite Spherical Joint (SPH - SPH)
    for jointCount=1:Joints.NCompSpherical
        [~,Jacobian,~,gamma,funCount] = Joint_CompSpherical([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
    end
    % For the Universal joints
    for jointCount=1:Joints.NUniversal
        [~,Jacobian,~,gamma,funCount] = Joint_Universal([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Universal,Flags); 
    end
    % For the Revolute joints
    for jointCount=1:Joints.NRevolute
        [~,Jacobian,~,gamma,funCount] = Joint_Revolute([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Revolute,Flags);
    end
    % For the Cylindrical joints
    for jointCount=1:Joints.NCylindrical
        [~,Jacobian,~,gamma,funCount] = Joint_Cylindrical([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
    end 
    % For the Translation joints
    for jointCount=1:Joints.NTranslation
        [~,Jacobian,~,gamma,funCount] = Joint_Translation([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Translation,Flags);
    end
    % For the Spherical Revolute Joints
    for jointCount=1:Joints.NSphRev
        [~,Jacobian,~,gamma,funCount] = Joint_CompSphRev([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
    end
    %For the Translational - Revolute joints
    for jointCount=1:Joints.NTraRev
        [~,Jacobian,~,gamma,funCount] = Joint_CompTraRev([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
    end
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [~,Jacobian,~,gamma,funCount] = Simple_Constraints([],Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    % For the Driving Constraints
    for jointCount=1:Joints.NDriver
        [~,Jacobian,~,Ctt,funCount] = Driver_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,time,driverfunctions); 
    end

%% Function Responsible for the Force Vectors    
    % Falta fazer o Force_TSpring que depende da atualização das variaveis
    [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction);
%% Assemblying the force vector and acceleration vector
    vetorg = Impose_Column(vetorg);
    gamma = Impose_Column(gamma);
    if Joints.NDriver >= 1
        Ctt = Impose_Column(Ctt);
        gamma = gamma - Ctt;
    end
    rhs = [vetorg;gamma];
    
%% Augmented Mass Matrix Assembly
    % Mass Matrix
    massmatrix = zeros(6*NBodies,6*NBodies); %Pre-Allocation
    for i = 1:NBodies
        Mass = Bodies(i).Mass;
        A = Bodies(i).A;
        Inertia = Bodies(i).Inertia;
        Irat = A*diag(Inertia)*A'; %Inertia convertion to Global Inertia Tensor (Nikra) - Rotated Theorem
        i1 = 6*(i-1)+1;
        massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
        massmatrix(i1+3:i1+5,i1+3:i1+5) = Irat; 
    end
    % Joining the new Jacobian with the Mass Matrix
    [a,~] = size(Jacobian);
    augmass = [massmatrix,-Jacobian';Jacobian,zeros(a,a)];
%% Solving the Initial Acceleration Problem System
    %Solvin the Linear Problem
    iapsol = augmass\rhs;
    %Allocating the solution to its respective vectors
    i1 = 6*NBodies;
    i2 = 6*NBodies + size(Jacobian,1);
    DynAcc = iapsol(1:i1,1);
    LagMulti = iapsol(i1+1:i2,1); 
%% Update and storage of the acceleration value for the t - deltat time.
    [Bodies] = UpdateAccelerations(DynAcc,NBodies,Bodies,SimType);
end