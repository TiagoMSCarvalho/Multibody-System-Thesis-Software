function [yd] = RKAuxFunction(DynAcc,NBodies,Bodies)
% This function is responsible for receiving the t0 initial accelerations
% and velocities, reallocate them to the yd (derivatives vector) that will
% be fed to ODE45 Runga-Kutta algorithm for integration.

%Calculus of the Euler Parameters derivatives to have qu in generalized coordinates
    qd = [];
    for i = 1:NBodies
        i1 = 7*(i-1)+1;
        pd = 0.5*Bodies(i).G'*Bodies(i).w; % Euler Parameters Derivatives
        qd(i1:i1+6,1) = [Impose_Column(Bodies(i).rd);Impose_Column(pd)];
    end
    yd = [qd;Impose_Column(DynAcc)];
end

