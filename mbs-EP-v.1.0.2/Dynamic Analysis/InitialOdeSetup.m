function [initial,t0,tf,DynAcc,Bodies] = InitialOdeSetup(NBodies,Bodies,tini,TimeStep,RunTime,dynfunc,Forces,Grav,UnitsSystem,ForceFunction,SimType)
%This function is used to setup the ode solver, initial solution q0 and
%times.

%% Stores the Times
    t0 = tini - TimeStep;
    tf = RunTime;
    time = t0;
    
%% Storage of the initial solution y0.
for i = 1:NBodies
        i1 = 7*(i-1)+1;
        vel(i1:i1+2,1) = Bodies(i).rd;
        pd = 0.5*Bodies(i).L'*Bodies(i).w;
        pd = Impose_Column(pd);
        vel(i1+3:i1+6,1) = pd;
        %Allocation of the initial positions
        i2 = 7*(i-1)+1;
        pos(i2:i2+6,1) = [Bodies(i).r;Bodies(i).p];
end
    vel = Impose_Column(vel);
    pos = Impose_Column(pos);
    initial = [pos;vel];
    
%% Calculates the first Dynamic Acceleration for storage.

%% Mass Matrix Assembly for Euler Parameters
    massmatrix = zeros(7*NBodies,7*NBodies); %Pre-Allocation
    for i = 1:NBodies
        Mass = Bodies(i).Mass;
        B = Bodies(i).L;
        Inertia = diag(Bodies(i).Inertia);
        Irat = 4.*B'*Inertia*B; %Nikra Article on the use of EP for 3D Dynamics
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
    coord = 7;
    [vetorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction,coord);

%% Solving First Iteration to start the Augmented Process
    [dim1,dim2] = size(massmatrix);
    wogmass = massmatrix(8:dim1,8:dim2);
    [dim3,~] = size(vetorg);
    wogvetor = vetorg(8:dim3,1);
    qddwog = lsqminnorm(wogmass,wogvetor);
    [dim4,~] = size(qddwog);
    qdd0(8:dim4+7,1) = qddwog;
    
    %% Transformation of the Acceleration Vector from 7 to 6 coordinates
    for i = 1:NBodies
        i1 = 6*(i-1)+1;
        i2 = 7*(i-1)+1;
        DynAcc(i1:i1+2,1) = qdd0(i2:i2+2,1);
        wd = 2*Bodies(i).L*qdd0(i2+3:i2+6,1); 
        DynAcc(i1+3:i1+5,1) = wd;
    end

%% Update and storage of the acceleration value for the t - deltat time.
    [Bodies] = UpdateAccelerations(DynAcc,NBodies,Bodies,SimType,[]);
end