function [Bodies,Points,CoM,DynAcc,it] = MBS_DynAnalysis(NBodies,Bodies,dynfunc,Joints,Forces,Points,CoM,t,TimeStep,Grav,SimType,UnitsSystem,it,driverfunctions,debugdata)
%Calls the functions needed to solve the Foward Dynamic Problem
    %% Runga-Kutta Pré Setup
    % Stores initial position,velocities and calculates the time interval for ode45
    [t0,tf,initial] = RKSetup (NBodies,Bodies,t,TimeStep);
    %Calculus of the dofs
    [debugdata] = SystemDofCalc(NBodies,Joints,debugdata);
    %Function that for the driver functions updates the velocity vector
    %called before due to Moment induced by Inertia due to the body
    %rotation.
    for jointCount = 1:Joints.NDriver
        [Bodies] = DynDriverVel(Bodies,Joints.Driver,jointCount,t0,driverfunctions);
    end
    % Function to calculate the Dynamic Initial Acceleration (2nd output is the lagrange multipliers).
    [DynAcc,~,~,Bodies] = DynInitialAccel(NBodies,Bodies,dynfunc,Joints,Forces,Grav,SimType,UnitsSystem,t0,driverfunctions,debugdata);
    %Function that retrieves the generalized coordinate Jacobian
    %[DCJac] = DCGenJac(NBodies,Bodies,Joints,driverfunctions,t0);
    % Update of the variables (Stores t - Timestep)
    [Points,CoM,it] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,DynAcc,it);
    %% Runga-Kutta Implementation RKAuxFunction, Aux function that feeds the inputs to ode45.
    opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
    rkfunc = @(t,y)RKAuxFunction(DynAcc,NBodies,Bodies);
    [vt,y] = ode45(rkfunc,[t0,tf],initial,opts);
    [a,~] = size(vt);
    y = Impose_Column(y(a,:));
    %% Direct Correction of the calculated qu and vu
    [~,~,Bodies] = RKDirectCorrection(y,NBodies,Bodies,Joints,SimType,driverfunctions,tf); %DCJac Eliminado.
end

