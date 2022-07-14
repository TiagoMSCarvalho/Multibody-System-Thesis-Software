function [Bodies,Points,CoM,DynAcc,it,debugdata] = MBS_DynAnalysis(NBodies,Bodies,dynfunc,Joints,Forces,Points,CoM,t,TimeStep,Grav,SimType,UnitsSystem,it,driverfunctions,debugdata,ForceFunction)
%Calls the functions needed to solve the Foward Dynamic Problem
    %% Integrator Pré Setup - Adams Moulton Bashforth ou Gear Method for stiff systems
    % Stores initial position,velocities and calculates the time interval for the integrator
    [t0,tf,initial] = RKSetup (NBodies,Bodies,t,TimeStep);
    
    % Function to calculate the Dynamic Initial Acceleration (2nd output is the lagrange multipliers).
    [DynAcc,~,~,Bodies] = DynInitialAccel(NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,t0,driverfunctions,debugdata,ForceFunction);
    
    % Update of the variables (Stores t - Timestep)
    [Points,CoM,it] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,DynAcc,it);
    
    
    opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
    rkfunc = @(t,y)RKAuxFunction(DynAcc,NBodies,Bodies);
    [vt,y] = ode113(rkfunc,[t0,tf],initial,opts);
    [a,~] = size(vt);
    y = Impose_Column(y(a,:));
    
    %% Update of the calculated qu and vu
    qu = y(1:7*NBodies,1);
    i1 = 7*NBodies + 1;
    i2 = 7*NBodies + 6*NBodies;
    vu = y(i1:i2,1);
    Bodies = UpdateBodyPostures(qu,NBodies,Bodies);
    Bodies = UpdateVelocities(vu,NBodies,Bodies,SimType);
end