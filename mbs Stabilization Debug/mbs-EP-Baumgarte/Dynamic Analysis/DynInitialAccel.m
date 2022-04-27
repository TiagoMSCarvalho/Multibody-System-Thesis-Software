function [DynAcc,LagMulti,Jacobian,Bodies] = DynInitialAccel(NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,time,driverfunctions,debugdata,ForceFunction,TimeStep)
%This function uses the inputs of initial position, initial velocities and
%forces to calculate the initial acceleration that will be fed to the
%Runge-Kutta ODE45 solver.

%% Set Up of the variables needed to construct the Matrix - Dynamic Modified Jacobian
% Pre Allocation of the q0 vector, for the calc of AGL
    q0 = CreateAuxiliaryBodyStructure(NBodies,Bodies);
% Pre Allocation of the qd0 vector, for the Baumgarte Stabilizaiton Method
    q0d = CreateVelocityVector(NBodies,Bodies);
% Calculation of the Matrixes A, G and L
    Bodies = DynCalcAGL(q0,NBodies,Bodies);
% Allocation of the Flags Struct
    % Flags.Dynamic, is a new field that was introduced to help to separate
    % the Kinematic Program from the Dynamic Program, and to keep the
    % program modular and of easy debug.
    Flags.Position = 1;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 1;
    Flags.AccelDyn = 1;
    funCount = 1;
    fun = [];
    Jacobian = [];
    gamma = zeros(debugdata(1).cdof - NBodies,1);
    Ct = zeros(debugdata(1).cdof - NBodies,1);
    Ctt = zeros(debugdata(1).cdof - NBodies,1);
% Assembly of the Jacobian and rhs acceleration vector.
    % For the Ground Constraints
    for jointCount=1:Joints.NGround
        [fun,Jacobian,~,gamma,funCount] = Ground_Constraints(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Ground,Flags);
    end
    % For the Spherical joints
    for jointCount=1:Joints.NSpherical
        [fun,Jacobian,~,gamma,funCount] = Joint_Spherical(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Spherical,Flags);
    end
    % For the Composite Spherical Joint (SPH - SPH)
    for jointCount=1:Joints.NCompSpherical
        [fun,Jacobian,~,gamma,funCount] = Joint_CompSpherical(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
    end
    % For the Universal joints
    for jointCount=1:Joints.NUniversal
        [fun,Jacobian,~,gamma,funCount] = Joint_Universal(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Universal,Flags); 
    end
    % For the Revolute joints
    for jointCount=1:Joints.NRevolute
        [fun,Jacobian,~,gamma,funCount] = Joint_Revolute(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Revolute,Flags);
    end
    % For the Cylindrical joints
    for jointCount=1:Joints.NCylindrical
        [fun,Jacobian,~,gamma,funCount] = Joint_Cylindrical(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
    end 
    % For the Translation joints
    for jointCount=1:Joints.NTranslation
        [fun,Jacobian,~,gamma,funCount] = Joint_Translation(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Translation,Flags);
    end
    % For the Spherical Revolute Joints
    for jointCount=1:Joints.NSphRev
        [fun,Jacobian,~,gamma,funCount] = Joint_CompSphRev(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
    end
    %For the Translational - Revolute joints
    for jointCount=1:Joints.NTraRev
        [fun,Jacobian,~,gamma,funCount] = Joint_CompTraRev(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
    end
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [fun,Jacobian,~,gamma,funCount] = Simple_Constraints(fun,Jacobian,[],gamma,funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    % For the Driving Constraints
    for jointCount=1:Joints.NDriver
        [fun,Jacobian,Ct,Ctt,funCount] = Driver_Constraints(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,time,driverfunctions); 
    end

%% Function Responsible for the Force Vectors    
    % Falta fazer o Force_TSpring que depende da atualização das variaveis
    [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction);
%% Stabilization Method - Baumgart Version
    alpha = 224;
    beta = 224;
%% Assemblying the force vector and acceleration vector
    vetorg = Impose_Column(vetorg);
    gamma = Impose_Column(gamma) - 2*alpha*(Jacobian*q0d) - beta^2*fun;
    if Joints.NDriver >= 1
        Ctt = Impose_Column(Ctt);
        gamma = gamma - Ctt - 2*alpha*(Jacobian*q0d + Ct) - beta^2*fun;
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