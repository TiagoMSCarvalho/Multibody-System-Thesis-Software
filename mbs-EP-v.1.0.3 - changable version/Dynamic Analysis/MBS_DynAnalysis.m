function [Bodies,Points,CoM,DynAcc,it,debugdata,pv,vv,timevector] = MBS_DynAnalysis(NBodies,Bodies,dynfunc,Joints,Forces,Points,CoM,TimeStep,Grav,SimType,UnitsSystem,it,driverfunctions,debugdata,ForceFunction,tini,RunTime)
%Calls the functions needed to solve the Foward Dynamic Problem
    % Prepares the initial conditions for the ode solver and calculates the
    % first acceleration for t=0, to store.
    [y0,t0,tf,DynAcc,Bodies] = InitialOdeSetup(NBodies,Bodies,tini,TimeStep,RunTime,dynfunc,Forces,Grav,UnitsSystem,ForceFunction,SimType);
    
    %Velocity and Pos Constraints Violation
    coord = 6;
    time = 0;
    pv = [];
    vv = [];
    [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,[],coord);
    Flags.Position = 1;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 1;
    Flags.AccelDyn = 0;
    [fun,Jacobian,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
    %display(fun);
    for i = 1:NBodies
           i1 = 6*(i-1)+1;
           qd(i1:i1+2,1) = Bodies(i).r;
           qd(i1+3:i1+5,1) = Bodies(i).w;
    end
    velvio = max(abs(Jacobian*qd));
    posvio = max(abs(fun));
%     velvio = (Jacobian*qd)'*(Jacobian*qd);
%     posvio = fun'*fun;
    
    % Update of the variables (Stores t - Timestep)
    [Points,CoM,it,pv,vv] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,DynAcc,it,pv,vv,posvio,velvio);
    
    tic;
    opts = odeset('RelTol',1e-6,'AbsTol',1e-6,'MaxStep',abs(t0-tf)*10^-1); 
    [timevector,y] = ode113(@(t,y)DynOdefunction(t,y,NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,driverfunctions,debugdata,ForceFunction),t0:TimeStep:tf,y0,opts);
    computationtime = toc;
    display(computationtime)
    [a,~] = size(timevector);
    
    %% Pre-setup 6 coordinates Dof
    coord = 6;
    [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,[],coord);
    massmatrix = zeros(6*NBodies,6*NBodies);
    
    %% All of this section can be stored in a different function for clarity.
    for k = 2:a
       %Define the time of calculus
       time = timevector(k,1);
       %Temporarily takes the y result of an integration step
       tempy = y(k,:);
       % Update Position before Acceleration Calculus
       position = tempy(1,1:7*NBodies);
       position = Impose_Column(position);
       Bodies = UpdateBodyPostures(position, NBodies, Bodies);
       % Update Velocities before the Acceleration Calculus
       velocity = tempy(1,(7*NBodies+1):(2*7*NBodies));
       velocity = Impose_Column(velocity);
       for i = 1:NBodies
           i1 = 7*(i-1)+1;
           Bodies(i).rd = velocity(i1:i1+2,1);
           Bodies(i).w = 2*Bodies(i).L*velocity(i1+3:i1+6,1);
           %% Augmented Mass Matrix Assembly
           Mass = Bodies(i).Mass;
           Inertia = Bodies(i).Inertia;
           I = diag(Inertia);
           i1 = 6*(i-1)+1;
           massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
           massmatrix(i1+3:i1+5,i1+3:i1+5) = I; 
       end
       % Flags for the Jacobian Calculus
       Flags.Position = 0;
       Flags.Jacobian = 1;
       Flags.Velocity = 0;
       Flags.Acceleration = 0;
       Flags.Dynamic = 1;
       Flags.AccelDyn = 1;
       gamma = zeros(debugdata(1).cdof,1);
       Ctt = zeros(debugdata(1).cdof,1);
       [~,Jacobian,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
       % Augmented Mass Matrix
       [b,~] = size(Jacobian);
       augmass = [massmatrix,Jacobian';Jacobian,zeros(b,b)];
       %% Function Responsible for the Force Vectors    
       [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction,coord);
       %% Assemblying the force vector and acceleration vector
       vetorg = Impose_Column(vetorg);
       gamma = Impose_Column(gamma);
       if Joints.NDriver >= 1
           Ctt = Impose_Column(Ctt);
           gamma = gamma - Ctt;
       end
       rhs = [vetorg;gamma]; %Erro force vector esta a ser calculado em 7 coord, resolver depois
       %% Calculus of the Acceleration vector
       %iapsol = lsqminnorm(augmass,rhs,1e-8);
       iapsol = augmass\rhs;
       %% Allocation of the Acceleration Results
       i3 = 6*NBodies;
       %i4 = 6*NBodies + size(Jacobian,1); Lagrangian
       DynAcc = iapsol(1:i3,1);
       %LagMulti = -iapsol(i3+1:i4,1);  %For later usage
       %% Update of the calculate Accelerations for previous storage
       [Bodies] = UpdateAccelerations(DynAcc,NBodies,Bodies,SimType,[]);
       %% Violations Constraints Graphics:
       for i = 1:NBodies
           i1 = 6*(i-1)+1;
           qd(i1:i1+2,1) = Bodies(i).r;
           qd(i1+3:i1+5,1) = Bodies(i).w;
       end
       velvio = max(abs((Jacobian*qd)));
       %velvio = (Jacobian*qd)'*(Jacobian*qd);
       Flags.Position = 1;
       Flags.Jacobian = 0;
       Flags.Velocity = 0;
       Flags.Acceleration = 0;
       Flags.Dynamic = 0;
       Flags.AccelDyn = 0;
       [fun,~,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
       %display(fun)
       posvio = max(abs(fun));
       %posvio = fun'*fun;
       %% Storage of this step calculus
       [Points,CoM,it,pv,vv] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,DynAcc,it,pv,vv,posvio,velvio);
       
    end  
end