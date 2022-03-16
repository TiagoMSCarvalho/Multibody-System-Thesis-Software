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


function [Bodies,Points,debugdata,it] = MultiBody_3D_Kinematic_Analysis(NBodies,Bodies,Joints,Points,t,it,opts,debugdata,ang,driverfunctions)

    %System Constrain check
    [debugdata] = SystemDofCalc(NBodies,Bodies,Joints,debugdata);
    % Process the input data
    [Bodies] = Position_Analysis(Joints,NBodies,Bodies,t,opts,ang,driverfunctions); %Até aqui os valores batem certo.
    [Bodies,Jacobian,vel,qd] = Velocity_Analysis(Joints,NBodies,Bodies,debugdata,ang,t,driverfunctions);
    [Bodies] = AccelerationPreDataTreat(qd,NBodies,Bodies);
    [Bodies,acc] = Acceleration_Analysis(Jacobian,Joints,NBodies,Bodies,debugdata,ang,t,driverfunctions);
    [Points,it] = KinDataStorage(Points,Bodies,Joints,vel,acc,it);

end