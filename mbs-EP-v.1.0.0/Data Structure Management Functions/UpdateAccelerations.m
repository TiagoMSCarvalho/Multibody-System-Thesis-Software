    %% Update Acceleration Function Details
%
% Inputs and Outputs:
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - qdd: vector with all the linear and angular accelerations of the bodies
%    in the same order as in the Bodies structure. This vector is used to
%    solve a matricial equation of accelerations.
%
% Objective of the Function:
%    This function updates the Bodies structure with the obtained
%    accelerations(from matricial equation solving) 

function Bodies = UpdateAccelerations(qdd,NBodies,Bodies,SimType)
% Update the velocities in the body information
if strcmp(SimType,"Kin") == 1
    for i=1:NBodies
        i1 = 7*(i-1)+1;
        Bodies(i).rdd = Impose_Column(qdd(i1:i1+2));
        Bodies(i).pdd = Impose_Column(qdd(i1+3:i1+6));
    end
elseif strcmp(SimType,"Dyn") == 1
    for i=1:NBodies
        i1 = 6*(i-1)+1;
        Bodies(i).rdd = Impose_Column(qdd(i1:i1+2));
        Bodies(i).wd = Impose_Column(qdd(i1+3:i1+5));
    end
end
end