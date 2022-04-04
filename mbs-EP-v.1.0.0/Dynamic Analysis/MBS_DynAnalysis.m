function [Bodies,Points,CoM,DynAcc,it] = MBS_DynAnalysis(NBodies,Bodies,Joints,Points,CoM,t,TimeStep,Grav,SimType,it)
%Calls the functions needed to solve the Foward Dynamic Problem

%% Runga-Kutta Pré Setup
% Stores initial position,velocities and calculates the time interval for ode45
[t0,tf,initial] = RKSetup (NBodies,Bodies,t,TimeStep);
% Function to calculate the Dynamic Initial Acceleration (2nd output is the lagrange multipliers.
[DynAcc,~,Jacobian,Bodies] = DynInitialAccel(NBodies,Bodies,Joints,Points,Grav,SimType);
% Update of the variables (Stores t - Timestep)
[Points,CoM,it] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,DynAcc,it);
%% Runga-Kutta Implementation RKAuxFunction, Aux function that feeds the inputs to ode45.
opts = odeset('RelTol',1e-4,'AbsTol',1e-4);
[~,y] = ode45(@RKAuxFunction,[t0,tf],initial,opts);
%% Direct Correction of the calculated qu and vu
[~,~,Bodies] = RKDirectCorrection(y,NBodies,Bodies,Jacobian,Joints,SimType);
end

