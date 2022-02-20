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
clear all
addpath('Data Structure Management Functions','Joints and Constraints','Kinematic Analysis','Auxiliary Functions','Excel Files','Post Processing Parameters Functions');
JointTypes = {'Spherical','Universal','Revolute','Cylindrical','Translation','Ground','Driver','Simple','Points'};

%file containing the suspension data:
filename = 'suspension_fst10e-one-wheel';

Points = [];
[Bodies, Joints, Motions] = PreDataProcessing(filename, JointTypes); %reads the excel file
NBodies = length(Bodies);
clc

Points_before = Points_of_Interest(1,Points,Bodies,Joints);

%% Fsolve Opts
%How to pick the solver!? Justification
opts=optimoptions('fsolve');
%opts.PlotFcn='optimplotfval';
opts.Algorithm='levenberg-marquardt';
opts.Diagnostics = 'off';
opts.Display = 'iter';
opts.ScaleProblem = 'Jacobian';
opts.UseParallel = false;
opts.FunctionTolerance = 1e-8;
opts.StepTolerance = 1e-8;
opts.InitDamping = 1e03;


%% Calcs
%this cycle iterates the position of every body in the program. t is given
%in seconds

TimeStep=cell2mat(Motions.TimeStep); %To use the Field is easier to pull it to a seperate variable in the m workspace
RunTime=cell2mat(Motions.RunTime);

for t=1:TimeStep:RunTime
    [Bodies,Points,dofinfo] = MultiBody_3D_Kinematic_Analysis(NBodies,Bodies,Joints,Points,t,t+1,opts);
end

[Points]=coordinate_transformation(Points,Motions);%coordinate
% transformation, in accordance to the drivers


% [SuspensionParameters]=sus_param_calculator(Points,Points_before,Joints,TimeStep);%computes the relevant
% aligment angles and values

[Points_array] = Gen_array_Points(Points, length(Points{1,1}));

%% Plots
SuspensionPlotter(Points, Points_before);%creates a plot with the suspension and respective members



