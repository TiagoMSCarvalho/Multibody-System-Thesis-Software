function [t0,tf,initial] = RKSetup (NBodies,Bodies,t,TimeStep)
% Stores initial position,velocities and calculates the time interval for ode45
    for i = 1:NBodies
        %Allocation of the initial velocities
        i1 = 6*(i-1)+1;
        vel(i1:i1+5,1) = [Bodies(i).rd;Bodies(i).w];
        %Allocation of the initial positions
        i2 = 7*(i-1)+1;
        pos(i2:i2+6,1) = [Bodies(i).r;Bodies(i).p];
    end
    vel = Impose_Column(vel);
    pos = Impose_Column(pos);
    initial = [pos;vel];
    % Store the time values for the integration interval
    t0 = t - TimeStep;
    tf = t;
end

