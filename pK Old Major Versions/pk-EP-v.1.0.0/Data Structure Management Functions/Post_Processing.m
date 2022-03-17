%% Post Processing Function Details
%
% Inputs and Outputs:
%    - numIteration: the number of the current iteration
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - Output_Bodies: a structure like Bodies but with the same information
%    for every iteration step
% Objective of the Function:
%    This function creates the Output_Bodies structure that contains all
%    the Bodies information for every iteration step. This function was
%    made considering that the relevant information for output is all the
%    positions, orientations and angular and linear velocities and
%    accelerations. To output also the rotation matrixes and derivative,
%    code needs to be added


function Output_Bodies = Post_Processing(numIteration,NBodies, Bodies, Output_Bodies)
    for k=1:NBodies
        % Position
        Output_Bodies(k).r{1}(numIteration) = Bodies(k).r(1);
        Output_Bodies(k).r{2}(numIteration) = Bodies(k).r(2);
        Output_Bodies(k).r{3}(numIteration) = Bodies(k).r(3);
        % Orientation
        Output_Bodies(k).theta{1}(numIteration) = Bodies(k).theta(1); 
        Output_Bodies(k).theta{2}(numIteration) = Bodies(k).theta(2); 
        Output_Bodies(k).theta{3}(numIteration) = Bodies(k).theta(3); 
        % Velocity
        Output_Bodies(k).rd{1}(numIteration) = Bodies(k).rd(1);
        Output_Bodies(k).rd{2}(numIteration) = Bodies(k).rd(2);
        Output_Bodies(k).rd{3}(numIteration) = Bodies(k).rd(3);
        % Angular Velocity
        Output_Bodies(k).thetad{1}(numIteration) = Bodies(k).thetad(1);
        Output_Bodies(k).thetad{2}(numIteration) = Bodies(k).thetad(2);
        Output_Bodies(k).thetad{3}(numIteration) = Bodies(k).thetad(3);
        % Acceleration
        Output_Bodies(k).rdd{1}(numIteration) = Bodies(k).rd(1);
        Output_Bodies(k).rdd{2}(numIteration) = Bodies(k).rd(2);
        Output_Bodies(k).rdd{3}(numIteration) = Bodies(k).rd(3);
        % Angular Acceleration
        Output_Bodies(k).thetadd{1}(numIteration) = Bodies(k).thetad(1);
        Output_Bodies(k).thetadd{2}(numIteration) = Bodies(k).thetad(2);
        Output_Bodies(k).thetadd{3}(numIteration) = Bodies(k).thetad(3);
    end
end 





