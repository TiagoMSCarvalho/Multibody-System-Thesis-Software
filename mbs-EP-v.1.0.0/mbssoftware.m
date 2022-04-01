%% ============================Multibody System Software=============================
%This is a multibody simulator, that can be used to solve problems involving Spatial Bodies.
%INPUTS:
%    - File:  template_mbs excel file must be edited as exemplified this is
%    crucial for the program to work correctly since it reads the excel ina
%    specific way.
%       - Joints - Defines the Joint Position and Information
%       - Joints Drivers - 
%       - Bodies - Inputs the Body information CoM position, orientation,
%       and Dynamic Inputs
%       - Simulation - Defines the simulation to be run, when the program
%       is finished a GUIDE (from App Designer) will replace this user
%       interface.
%    - The program uses Euler Parameters to determine the orientation of
%    each body and to avoid the singularities from the Bryant Angles.
%OUTPUTS:
%   Kinematic Analysis: (previous pK)
%       - The program gives the Position, Velocity and Acceleration of the
%       Points of Interest in response to a motion forced on one of its
%       bodies.
%       - Units: Positions in mm and rad, velocities in mm/s or rad/s,
%       accelerations in mm/s^2 and rad/s^2
%   Dynamic Analysis:
%       - The program gives the final position of the bodies in response to
%       a force input;

%% Workspace Prep
clc
addpath(genpath(pwd))
clearvars %clear all decreases code performance
addpath('Data Structure Management Functions','Joints and Constraints','Kinematic Analysis','Dynamic Analysis','Auxiliary Functions','Excel Files','Force Elements','Post Processing Parameters Functions');
JointTypes = {'Spherical','Universal','Revolute','Cylindrical','Translation','Ground','Driver','Simple','Points'};
ForcesTypes = {'Spring','TSpring','Damper'};

%file containing the suspension data:
filename = 'template_mbs';

%% Retrive Information from the Excel
Points = []; %Array to save the points during each iteration, lines are the points, col their value to each iteration
[Bodies,Joints,SimParam,Grav,debugdata,ang,driverfunctions] = PreDataProcessing(filename,JointTypes); %reads the excel file
NBodies = length(Bodies);
clc

%% Set up the Simulation (Run Time/Step/Type) and Storage Initial Data
TimeStep=cell2mat(SimParam.TimeStep); %To use the Field is easier to pull it to a seperate variable in the m workspace
RunTime=cell2mat(SimParam.RunTime);
Ntotit = RunTime/TimeStep;
tini = TimeStep; %Initial Time 0s + Initial Time, q0( pos for t = 0 s) is already stored.
SimType = string(SimParam.SimulationType);

it = 1;

[Points,it] = KinDataStorage(Points,Bodies,Joints,[],[],it);

%% Solvers
if strcmp(SimType,"Kin") == 1
%% Kinematic Solver    
% Fsolve Opts (Justification is Found in The Mendeley) - Kin
    opts=optimoptions('fsolve');
    opts.Algorithm='levenberg-marquardt';
    opts.Diagnostics = 'off';
    opts.Display = 'iter';
    opts.ScaleProblem = 'Jacobian';
    opts.UseParallel = false;
    opts.FunctionTolerance = 1e-6;
    opts.StepTolerance = 1e-6;
    
 % Calcs (t in seconds) 
    for t=tini:TimeStep:RunTime
        [Bodies,Points,debugdata,it] = MultiBody_3D_Kinematic_Analysis(NBodies,Bodies,Joints,Points,t,it,opts,debugdata,ang,driverfunctions);
    end
%% Dynamic Solver
elseif strcmp(SimType,"Dyn") == 1
    for t = tini:TimeStep:RunTime
%NOTE: Runga-Kutta will be use to perform the Velocity and Position
%Analysis then the Direct Correction will be implemented see:
%"Development and Appplication of a Computational Dynamic and Kinematic Constrained Multibody System Simulations" page 77
%"On the constrains violation in forward dynamics of multibody systems pg 18
        %% Runga-Kutta Pré Setup
        % Stores initial position,velocities and calculates the time interval for ode45
        [t0,tf,initial] = RKSetup (NBodies,Bodies,t,TimeStep);
        % Function to calculate the Dynamic Initial Acceleration
        [DynAcc,LagMulti,Jacobian] = DynInitialAccel(NBodies,Bodies,Joints,Points,Grav);
        %% Runga-Kutta Implementation RKAuxFunction, Aux function that feeds the inputs to ode45.
        opts = odeset('RelTol',1e-4,'AbsTol',1e-4);
        [steps,y] = ode45(@RKAuxFunction,[t0,tf],initial,opts);
        %% Direct Correction of the calculated qu and vu
        [qc,vc] = RKDirectCorrection(y,NBodies,Bodies,Jacobian,Joints);
        % Update of the variables
        
    end
end

