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
addpath('Data Structure Management Functions','Joints and Constraints','Kinematic Analysis','Dynamic Analysis','Auxiliary Functions','Excel Files','Force Elements','Post Processing');
JointTypes = {'Spherical','CompSpherical','Universal','Revolute','Cylindrical','Translation','SphRev','TraRev','Ground','Driver','Simple','Points'};
ForcesTypes = {'Spring','TSpring','Damper','Actuator'};

%file containing the suspension data:
filename = 'flyball_governor_damper_spring';

%% Retrive Information from the Excel
%Array to save the points during each iteration, lines are the points, col their value to each iteration
Points = [];
CoM = [];
[Bodies,Joints,Forces,SimParam,Grav,UnitsSystem,debugdata,ang,driverfunctions,dynfunc,ForceFunction,GraphicsType,BodiesGraph,PointsGraph] = PreDataProcessing(filename,JointTypes,ForcesTypes); %reads the excel file
NBodies = length(Bodies);
clc

%% Set up the Simulation (Run Time/Step/Type) and Storage Initial Data
TimeStep=cell2mat(SimParam.TimeStep); %To use the Field is easier to pull it to a seperate variable in the m workspace
RunTime=cell2mat(SimParam.RunTime);
Ntotit = RunTime/TimeStep;
tini = TimeStep; %Initial Time 0s + Initial Time, q0( pos for t = 0 s) is already stored.
SimType = string(SimParam.SimulationType);
it = 1;

%% Solvers
if strcmp(SimType,"Kin") == 1
% Storage of the first values for the Kinematic Analysis
    [CoM,Points,it] = KinDataStorage(CoM,Points,NBodies,Bodies,Joints,[],[],it);    
%% Kinematic Solver    
% Fsolve Opts (Justification is Found in The Mendeley) - Kin
    opts=optimoptions('fsolve');
    opts.Algorithm='trust-region-dogleg';
    opts.Diagnostics = 'off';   
    opts.Display = 'iter';
    opts.ScaleProblem = 'Jacobian';
    opts.UseParallel = false;
    opts.FunctionTolerance = 1e-6;
    opts.StepTolerance = 1e-6;
    
 % Calcs (t in seconds) 
    for t=tini:TimeStep:RunTime
        [Bodies,Points,CoM,debugdata,it] = MultiBody_3D_Kinematic_Analysis(NBodies,Bodies,Joints,Points,t,it,opts,debugdata,ang,driverfunctions,SimType,CoM);
    end
%% Dynamic Solver
elseif strcmp(SimType,"Dyn") == 1
    %% Initial Conditions Consistency Check
    [Bodies] = InitialConditionCorr(NBodies,Bodies,Joints,SimType);
    %% Calling the function that will solve the dynamic problem.
    [Bodies,Points,CoM,DynAcc,it,debugdata] = MBS_DynAnalysis(NBodies,Bodies,dynfunc,Joints,Forces,Points,CoM,TimeStep,Grav,SimType,UnitsSystem,it,driverfunctions,debugdata,ForceFunction,tini,RunTime);
    %% Final Plots
    DynPlots(GraphicsType,BodiesGraph,PointsGraph,RunTime,TimeStep,CoM,Points);
end    