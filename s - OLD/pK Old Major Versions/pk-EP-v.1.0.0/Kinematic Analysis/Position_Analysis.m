%% Position Analysis Function Details
%
% Inputs and Outputs:
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - Joints: structure with all the information relative to the joints
%    and constraints divided in fields correspondent to each type of
%    joint/constraint
%
% Objective of the Function:
%    This function objective is to solve a system of equations to get the
%    position and orientation of all the bodies, meeting the criteria
%    imposed by the joints and constraints. A set of functions is
%    calculated and concatenated to use on the function fsolve. In the end,
%    the Bodies' information is updated for the next step

function [Bodies,fval,exitflag] = Position_Analysis(Joints,NBodies,Bodies,t,opts)

    % Evaluate the Position Constraint Equations
    % Function that evaluates the position constraint equations
    % Initialize variables with new position information and
    
    q0 = CreateAuxiliaryBodyStructure(NBodies,Bodies); %Pre-Allocation of q0 vector
    
    % Form the position analysis constraint equations
    Flags.Position = 1;
    Flags.Jacobian = 0;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;

    constrainsys = @(q)GenerateFunctions(q,Joints,NBodies,Bodies,Flags,t); %Handle Input @ (arglist)
    [q] = fsolve(constrainsys,q0,opts); %1st Func, 2nd Initial Guess, 3rd Solver Options
    Bodies = UpdateBodyPostures(q, NBodies, Bodies);
    
    disp(q)
end

