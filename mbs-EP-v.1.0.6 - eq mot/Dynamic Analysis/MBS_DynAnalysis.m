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
    Flags.Eqmotion = 0;
    
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
    initime = cputime;
    opts = odeset('RelTol',1e-6,'AbsTol',1e-6,'MaxStep',5*10^-3); %'MaxStep',5*10^-4
    [timevector,y] = ode113(@(t,y)DynOdefunction(t,y,NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,driverfunctions,debugdata,ForceFunction),t0:TimeStep:tf,y0,opts);
    computationtime = toc;
    fintime = cputime;
    cpu = fintime-initime;
    display(computationtime)
    display(cpu)
    [a,~] = size(timevector);
    

    
    %% All of this section can be stored in a different function for clarity. Mudar para parametros de Euler para tirar as forças como deve ser.
    for k = 2:a
       
       %display(k);
       coord = 7;
       [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,[],coord);
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
           Bodies(i).w = 2*Bodies(i).G*velocity(i1+3:i1+6,1);
           Bodies(i).wl = Bodies(i).w; 
       end
       
       massmatrix = zeros(7*NBodies,7*NBodies); %Pre-Allocation
       
       qi = CreateAuxiliaryBodyStructure(NBodies,Bodies);
       Bodies = DynCalcAGL(qi,NBodies,Bodies);
       [Bodies] = DynIAGdLdcalc(NBodies,Bodies);
       
       for i = 1:NBodies
           Mass = Bodies(i).Mass;
           B = Bodies(i).L;
           Inertia = diag(Bodies(i).Inertia);
           Irat = 4*B'*Inertia*B; %Nikra Article on the use of EP for 3D Dynamics
           i1 = 8*(i-1)+1;
           massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
           massmatrix(i1+3:i1+6,i1+3:i1+6) = Irat; 
           massmatrix(i1+3:i1+6,i1+7) = 2*Bodies(i).p;
           massmatrix(i1+7,i1+3:i1+6) = 2*Bodies(i).p';
       end
       
       
       %Jacobian Matrix
       Flags.Position = 0;
       Flags.Jacobian = 0;
       Flags.Velocity = 0;
       Flags.Acceleration = 1;
       Flags.Dynamic = 0;
       Flags.AccelDyn = 0;
       Flags.Eqmotion = 1;
       coord = 8;
       [~,Jacobian,~,gamma] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
    
       % Augmented Mass Matrix
       [b,~] = size(Jacobian);
       augmass = [massmatrix,Jacobian';Jacobian,zeros(b,b)];
       %% Function Responsible for the Force Vectors   
       [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction,coord);
       coord = 7;
       %% Assemblying the force vector and acceleration vector
       vetorg = Impose_Column(vetorg);
       gamma = Impose_Column(gamma);
       rhs = [vetorg;gamma];
       %% Calculus of the Acceleration vector
       iapsol = lsqminnorm(augmass,rhs,1e-15);
       %% Allocation of the Acceleration Results
       i3 = 8*NBodies;
       DynAcc = iapsol(1:i3,1);
       
       for i = 1:NBodies
           i1 = 8*(i-1)+1;
           Bodies(i).rdd = iapsol(i1:i1+2,1);
           Bodies(i).wd = 2*Bodies(i).L*iapsol(i1+3:i1+6,1);
       end   

       coord = 6;
       [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,[],coord);
       
       
       %% Violations Constraints Graphics;
       % Flags for the Jacobian Calculus
       Flags.Position = 0;
       Flags.Jacobian = 1;
       Flags.Velocity = 0;
       Flags.Acceleration = 0;
       Flags.Dynamic = 1;
       Flags.AccelDyn = 1;
       Flags.Eqmotion = 0;
       
       [~,Jacobian,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
       
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
       Flags.Eqmotion = 0;
       
       [fun,~,~,~] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord,time);
       %display(fun)
       posvio = max(abs(fun));
       %posvio = fun'*fun;
       %% Storage of this step calculus
       [Points,CoM,it,pv,vv] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,DynAcc,it,pv,vv,posvio,velvio);
    end  
end