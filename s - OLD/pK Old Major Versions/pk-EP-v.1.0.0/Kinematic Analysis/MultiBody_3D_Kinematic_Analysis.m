%Inputs and Outputs:
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - Joints: structure with all the information relative to the joints
%    and constraints divided in fields correspondent to each type of
%    joint/constraint
%    -Points: Points previously attained
%    -t: time in seconds [s]
%    -nIteration: number of the iteration

% Objective of the Function:
%    This function's objective is to to call the funtions that compute the
%    points, then updating their position


function [Bodies, Points,dofinfo] = MultiBody_3D_Kinematic_Analysis(NBodies,Bodies,Joints,Points,t,nIteration, opts)

    %System Constrain check
    [dofinfo] = SystemDofCalc(NBodies,Bodies,Joints);
    % Process the input data
    Bodies = Position_Analysis(Joints,NBodies,Bodies,t, opts);
    [Bodies,Jacobian,qd] = Velocity_Analysis(Joints,NBodies,Bodies,dofinfo);
    [Bodies] = AccelerationPreDataTreat(qd,NBodies,Bodies);
    [Bodies] = Acceleration_Analysis(Jacobian,Joints,NBodies,Bodies);
    Points = Points_of_Interest(nIteration,Points,Bodies,Joints);

end