function [DynAcc,LagMulti,Jacobian,Bodies] = DynInitialAccel(NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,time,driverfunctions,debugdata,ForceFunction)
%This function uses the inputs of initial position, initial velocities and
%forces to calculate the initial acceleration that will be fed to the
%Integrator.

%% Set Up of the variables needed to construct the Matrix - Dynamic Modified Jacobian
% Pre Allocation of the q0 vector, for the calc of AGL
    q0 = CreateAuxiliaryBodyStructure(NBodies,Bodies);
% Calculation of the Matrixes A, G and L
    Bodies = DynCalcAGL(q0,NBodies,Bodies);
%Calculus of the dofs
    coord = 7;
    [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,[],coord);
    
%% Assembly of the Invariable Matrices in the Augmented Lagrangian Formula.
    % The process to obtain Position Matrix will be separated from the
    % Jacobian to avoid funcount conflitcs
    
    %Position Matrix
    Flags.Position = 1;
    Flags.Jacobian = 0;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    
    [fun,~,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions);
    
    %Jacobian Matrix
    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    
    [~,Jacobian,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions);
    
%% Mass Matrix Assembly for Euler Parameters
    massmatrix = zeros(7*NBodies,7*NBodies); %Pre-Allocation
    for i = 1:NBodies
        Mass = Bodies(i).Mass;
        B = Bodies(i).L;
        Inertia = Bodies(i).Inertia;
        Irat = 4*B'*diag(Inertia)*B; %Nikra Article on the use of EP for 3D Dynamics
        i1 = 7*(i-1)+1;
        massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
        massmatrix(i1+3:i1+6,i1+3:i1+6) = Irat; 
    end
    
%% Definition of the Velocity Vector from 6 coordinates to 7 coordinates
    for i = 1:NBodies
        i1 = 7*(i-1)+1;
        qd(i1:i1+2,1) = Bodies(i).rd;
        pd = 0.5*Bodies(i).L'*Bodies(i).w;
        pd = Impose_Column(pd);
        qd(i1+3:i1+6,1) = pd;
    end
    
%% Function Responsible for the Force Vectors
    [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction);

%% Solving First Iteration to start the Augmented Process
    qdd0 = massmatrix\vetorg;
    
%% Define Augmented Lagrangian Penalty Parameters
    %Values taken from Paulo Flores Art on Constraints
    alpha = 1*10^7;
    omega = 10;
    mu = 1;
    
%% Solving the Augmented Lagrangian Formula Iterative Formula
    delta = 1;
    qddi = qdd0;
    lagit = 1; % Augmented Lagrangian Formula Iteration Numver
    % Flags to retrieve gamma
    Flags.Position = 0;
    Flags.Jacobian = 0;
    Flags.Velocity = 1;
    Flags.Acceleration = 1;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    while delta >1*10^-3 && lagit < 20 % Second Condition to avoid infinite loops
        if lagit > 1
           qddi = qddi1; 
        end
        [~,~,niu,gamma] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions);
        lhslag = massmatrix + Jacobian'*alpha*Jacobian;
        rhslag = massmatrix*qddi + Jacobian'*alpha*(gamma - 2*omega*mu*(Jacobian*qd - niu) - omega^2*fun);
        qddi1 = lhslag\rhslag;
        delta = qddi1 - qddi;
        lagit = lagit + 1; %Iteration Counter
    end

%% Transformation of the Acceleration Vector from 7 to 6 coordinates
    for i = 1:NBodies
        i1 = 6*(i-1)+1;
        i2 = 7*(i-1)+1;
        DynAcc(i1:i1+2,1) = qddi(i2:i2+2,1);
        wd = 2*Bodies(i).L*qddi(i2+3:i2+6,1);
        DynAcc(i1+3:i1+5,1) = wd;
    end

%% Update and storage of the acceleration value for the t - deltat time.
    [Bodies] = UpdateAccelerations(DynAcc,NBodies,Bodies,SimType);
    
%% Augmented Mass Matrix Assembly for the Lagrange Calculation (6 Coord)
    % Mass Matrix
    massmatrix = zeros(6*NBodies,6*NBodies); %Pre-Allocation
    for i = 1:NBodies
        Mass = Bodies(i).Mass;
        A = Bodies(i).A;
        Inertia = Bodies(i).Inertia;
        Irat = A*diag(Inertia)*A'; %Inertia convertion to Global Inertia Tensor (Nikra) - Rotated Theorem (Paulo Flores)
        i1 = 6*(i-1)+1;
        massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
        massmatrix(i1+3:i1+5,i1+3:i1+5) = Irat; 
    end
%% Pre-setup 6 coordinates Dof
%Calculus of the dofs
    coord = 6;
    [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,[],coord);
%% 6 coordinate Jacobian for the Lagrange Multipliers
    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 1;
    Flags.AccelDyn = 0;
    
    [~,Jacobian,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions);
%% Lagrange Mutipliers Calculus
    rhs = vetorg - massmatrix*DynAcc;
    LagMulti = (Jacobian')\rhs;
end