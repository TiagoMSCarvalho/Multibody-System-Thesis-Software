%% Update Velocity Function Detai1s
%
% Inputs and Outputs:
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - qd: vector with all the linear and angular velocities of the bodies
%    in the same order as in the Bodies structure. This vector is used to
%    solve a matricial equation of velocities.
%
% Objective of the Function:
%    This function updates the Bodies structure with the obtained
%    velocities (from matricial equation solving) 

function Bodies = UpdateVelocities(qd,NBodies,Bodies)
% Update the velocities in the body information
for i=1:NBodies
    i1 = 7*(i-1)+1;
    Bodies(i).rd = Impose_Column(qd(i1:i1+2));
    Bodies(i).pd = Impose_Column(qd(i1+3:i1+6));
end
end
