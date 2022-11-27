function [yd] = DynOdefunction(t,y,NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,driverfunctions,debugdata,ForceFunction)
%This function uses the inputs of initial position, initial velocities and
%forces to calculate the initial acceleration that will be fed to the
%Integrator.

%% Update the Structs with the variables that are given by the odesolver.
    time = t;
%Position
    Bodies = UpdateBodyPostures(y(1:7*NBodies), NBodies, Bodies);

%% Set Up of the variables needed to construct the Matrix - Dynamic Modified Jacobian
% Pre Allocation of the q0 vector, for the calc of AGL
    q0 = CreateAuxiliaryBodyStructure(NBodies,Bodies);    
% Calculation of the Matrixes A, G and L
    Bodies = DynCalcAGL(q0,NBodies,Bodies);
% Calculus of the dofs
    coord = 7;
    [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,[],coord);
%% Velocity Update

    for i = 1:NBodies
       i1 = 7*(i-1)+1;
       i2 = 7*(i-1) + 1 + 7*NBodies;
       Bodies(i).rd = y(i2:i2+2,1);
       pd = y(i2+3:i2+6,1);
       Bodies(i).w = 2*Bodies(i).L*pd;
       qd(i1:i1+2,1) = y(i2:i2+2,1);
       qd(i1+3:i1+6,1) = pd;
    end
    
% Calculus of the G and L matrices derivatives
    [Bodies] = DynIAGdLdcalc(NBodies,Bodies);
% Change w field to allow Kinematic Functions to Run on the RHS Accel (Can
% be Optimized)

    for i = 1:NBodies
       Bodies(i).wl = Bodies(i).w; 
    end
    
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
    Flags.Eqmotion = 0;
    
    [fun,~,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
    
    %Jacobian Matrix
    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    Flags.Eqmotion = 0;
    
    [~,Jacobian,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
    
%% Mass Matrix Assembly for Euler Parameters
    massmatrix = zeros(7*NBodies,7*NBodies); %Pre-Allocation
    for i = 1:NBodies
        Mass = Bodies(i).Mass;
        B = Bodies(i).L;
        Inertia = diag(Bodies(i).Inertia);
        Irat = 4*B'*Inertia*B; %Nikra Article on the use of EP for 3D Dynamics
        i1 = 7*(i-1)+1;
        massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
        massmatrix(i1+3:i1+6,i1+3:i1+6) = Irat; 
    end
    
% %% Definition of the Velocity Vector from 6 coordinates to 7 coordinates
%     for i = 1:NBodies
%         i1 = 7*(i-1)+1;
%         qd(i1:i1+2,1) = Bodies(i).rd;
%         pd = 0.5*Bodies(i).L'*Bodies(i).w;
%         pd = Impose_Column(pd);
%         qd(i1+3:i1+6,1) = pd;
%     end
    
%% Function Responsible for the Force Vectors
    [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction,coord);

%% Solving First Iteration to start the Augmented Process
%     [dim1,dim2] = size(massmatrix);
%     [dim3,~] = size(vetorg);
    %qdd0(8:dim3,1) = lsqminnorm(massmatrix(8:dim1,8:dim2),vetorg(8:dim3,1),1e-8);    
    qdd0 = lsqminnorm(massmatrix,vetorg,1e-10);

%% Define Augmented Lagrangian Penalty Parameters
    %Values taken from Paulo Flores Art on Constraints
    jdim1 = size(Jacobian,1);
% For single pendulum
%     alpha = 1*10^8*eye(jdim1);
%     omega = 500*eye(jdim1);
%     mu = 10*eye(jdim1);
% For NBar Mechanism 
%     alpha = 1e10*eye(jdim1);
%     omega = 2000*eye(jdim1);
%     mu = 1*eye(jdim1);
% % For Flyball Governor
%     alpha = 1e7*eye(jdim1);
%     omega = 1000*eye(jdim1);
%     mu = 1*eye(jdim1);
% For Nbar_SpringLinkage
%     alpha = 1e8*eye(jdim1);
%     omega = 250*eye(jdim1);
%     mu = 1*eye(jdim1);
% For Bricards Mechanism
%     alpha = 1e7*eye(jdim1);
%     omega = 1000*eye(jdim1);
%     mu = 1*eye(jdim1);
% For Andrews Mechanism
%     alpha = 10^2*eye(jdim1);
%     omega = 10^5*eye(jdim1);
%     mu = 1*eye(jdim1);


    alpha = 10^7*eye(jdim1);
    omega = 10^2*eye(jdim1);
    mu = 1*eye(jdim1);
    
%% Solving the Augmented Lagrangian Formula Iterative Formula
    alf = 'alfon';
    deltamax = 1;
    qddi = qdd0;
    lagit = 0; % Augmented Lagrangian Formula Iteration Number
    lhslag = massmatrix + Jacobian'*alpha*Jacobian;
    
    % Flags to retrieve gamma
    Flags.Position = 1;
    Flags.Jacobian = 0;
    Flags.Velocity = 1;
    Flags.Acceleration = 1;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    Flags.Eqmotion = 0;
    while deltamax > 1e-4
        if lagit >= 1
           qddi = qddi1; 
        end
        [~,~,niu,gamma] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
        rhslag = massmatrix*qddi + Jacobian'*alpha*(gamma -  2*omega*mu*(Jacobian*qd - niu) - omega^2*fun);
        qddi1 = gaussian_elimination(lhslag,rhslag);
        qdd = qddi1;
        deltaqdd = qddi1 - qddi;
        deltamax = abs(max(deltaqdd));
        [Bodies] = UpdateAccelerations(qddi1,NBodies,Bodies,SimType,alf);
        lagit = lagit + 1; %Iteration Counter
    end
    yd = [qd;qdd];
    
    %lagrange = Jacobian'\(vetorg-massmatrix*qdd)
    %forces = massmatrix*qdd;

display(time);
end

