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
       i2 = 7*(i-1) + 1 + 7*NBodies;
       Bodies(i).rd = y(i2:i2+2,1);
       pd = y(i2+3:i2+6,1);
       Bodies(i).w = 2*Bodies(i).L*pd;
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
    
    [fun,~,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
    
    %Jacobian Matrix
    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    
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
    
%% Definition of the Velocity Vector from 6 coordinates to 7 coordinates
    for i = 1:NBodies
        i1 = 7*(i-1)+1;
        qd(i1:i1+2,1) = Bodies(i).rd;
        pd = 0.5*Bodies(i).L'*Bodies(i).w;
        pd = Impose_Column(pd);
        qd(i1+3:i1+6,1) = pd;
    end
    
%% Function Responsible for the Force Vectors
    [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction,coord);

%% Define Augmented Lagrangian Penalty Parameters

    alfa = 10^4;
    beta = 10^4;

    %% Baumgarte Stabilization
    % Define Baumgarte Penalty Parameters
    dim1 = size(Jacobian,1);
    dim2 = size(Jacobian,2);
    
    %Leading Matrix
    leadingmatrix = [massmatrix, Jacobian'; Jacobian, zeros(dim1,dim1)];
    
    %gamma retrieval
    % Flags to retrieve gamma
    Flags.Position = 1;
    Flags.Jacobian = 0;
    Flags.Velocity = 1;
    Flags.Acceleration = 1;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    
    [~,~,~,gamma] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
    
    baumgartevector = Impose_Column(gamma - 2*alfa*(Jacobian*qd) - beta^2*fun);
    rhsvector = [vetorg;baumgartevector];
    
    
    baumgartesolution = linsolve(leadingmatrix,rhsvector);
    %baumgartesolution = regress(rhsvector',leadingmatrix','TSVD');
    
    qdd = baumgartesolution(1:dim2);
    %qdd = qdd';
    
    alf = 'alfon';
    
    [Bodies] = UpdateAccelerations(qdd,NBodies,Bodies,SimType,alf);
    
    yd = [qd;qdd];

display(time);
end

