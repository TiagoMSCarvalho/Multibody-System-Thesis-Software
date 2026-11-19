%% ============================perfectKinematics=============================
%This is a multibody simulator, applied to a suspension.
%INPUTS:
%    - File: all the information of the MultiBody System. This file must be
%    in the program's folder. It is read in a certain way by the program,
%    so it is crucial to be edited as the example.xlsx is.
%    The file provides all the bodies, joints and motions data needed.
%    The dimensions are given in mm, speeds in mm/s. Accelereations are
%    disregarded
%    The directions are:
%    1- x-axis
%    2- y-axis
%    3- z-axis (velocity down?)
%    4- Roll
%    5- Pitch
%    6- Yaw
%    %Note: Points of interest should not be changed
%    - User Inputs: choose the motions wanted, and their respective value
%
%    OUTPUTS:
%    -Points: The progression of the points in the "points of interest" in the
%    excel
%    -Plots: Plots showing the suspension (simplified) components
% Objective of the Program:
%    This program objective is to calculate needed relations to then
%    evaluate camber, toe, etc.

%% Workspace Prep
clc
addpath(genpath(pwd))
clearvars %clear all decreases code performance
addpath('Data Structure Management Functions','Joints and Constraints','Kinematic Analysis','Auxiliary Functions','Excel Files','Post Processing Parameters Functions');
JointTypes = {'Spherical','Universal','Revolute','Cylindrical','Translation','Ground','Driver','Simple','Points'};

%file containing the suspension data:
filename = 'crank_slide_2D_Nikra';

Points = []; %Array to save the points during each iteration, lines are the points, col their value to each iteration
[Bodies,Joints,Motions,debugdata,ang,driverfunctions] = PreDataProcessing(filename,JointTypes); %reads the excel file
NBodies = length(Bodies);
clc

it = 1;

[Points,it] = KinDataStorage(Points,Bodies,Joints,[],[],it);

%% Fsolve Opts
%How to pick the solver!? Justification
opts=optimoptions('fsolve');
%opts.PlotFcn='optimplotfval';  
opts.Algorithm='trust-region-dogleg';
%opts.Algorithm ='levenberg-marquardt';
%opts.Algorithm = 'trust-region';
opts.Diagnostics = 'off';
opts.Display = 'iter';
opts.ScaleProblem = 'Jacobian';
opts.UseParallel = false;
opts.FunctionTolerance = 1e-6;
opts.StepTolerance = 1e-6;
%opts.InitDamping = 1e03; %-> Taken out after some debug it was causing the
%program to fully converg on the first iteration since 1000 damping was too
%high for following iterations


%% Calcs
%this cycle iterates the position of every body in the program. t is given
%in seconds

TimeStep=cell2mat(Motions.TimeStep); %To use the Field is easier to pull it to a seperate variable in the m workspace
RunTime=cell2mat(Motions.RunTime);
Ntotit = RunTime/TimeStep;
tini = TimeStep; %Initial Time 0s + Initial Time, q0( pos for t = 0 s) is already stored.

for t=tini:TimeStep:RunTime
    tic;
    [Bodies,Points,debugdata,it] = MultiBody_3D_Kinematic_Analysis(NBodies,Bodies,Joints,Points,t,it,opts,debugdata,ang,driverfunctions);
    computationtime = toc;
    display(computationtime);
end

%% POST-PROCESSING

%[Points]=coordinate_transformation(Points,Motions);%coordinate
%transformation, in accordance to the drivers -> Confuso.


% [SuspensionParameters]=sus_param_calculator(Points,Points_before,Joints,TimeStep);%computes the relevant
% aligment angles and values

% [Points_array] = Gen_array_Points(Points, length(Points{1,1}));

%% Plots
%SuspensionPlotter(Points, Points_before);%creates a plot with the suspension and respective members
Plotter(Points,Joints,it)