%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inputs and Outputs:
%    - fun : the fsolve tool will define value for the variables so that each
%    expression is equal to zero. The f variable carries all the values of
%    the expressions (f will be all zeros when solved)
%    - Jacobian : the Jacobian Matrix (check documentation for more
%      information)
%    - niu : right-hand-side of the velocity equations (check documentation
%      for more information)
%    - gamma : right-hand-side of the acceleration equations (check documentation
%      for more information)
%    - funCount: the equation/function counter
%    - jointCount: current Joint number on the structure Revolute
%    - Bodies: all the information of the MultiBodies System
%    - Driver: all the information of the Driver constraints present in
%    the multiBodies system
%    - Flags: 4 flags that determine which values to be calculated (each
%    flag for one of the 4 first inputs). 1 means to calculate
%    - t: time of the simuation
%
% Objective of the Function:
%    This functions objective is calculate the values of the expressions
%    the formulate each specific joints, calculate its Jacobian and
%    the right-hand-side of both velocity and acceleration equations. Each
%    calculation will only be made if the correspondent Flag value is 1.
%    Note that the multiBodies systems' Jacobian is defined with all the joints  
%    and constraints, which means, each joint just defines a few lines of
%    this matrix and the values that are calculated are only in the columns
%    correspondent to the bodies involved


function [fun,Jacobian,niu,gamma,funCount] = Driver_Constraints(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Driver,Flags,t,ang)
i = Driver(jointCount).Body;  
direction = Driver(jointCount).direction;      
pos0 = Driver(jointCount).pos0;
v0 = Driver(jointCount).v0; 
a0 = Driver(jointCount).a0;

% Form the position constraint equations
if( Flags.Position == 1)
    if direction == 1
        x = Bodies(i).r(direction);
        fun(funCount,1) = x - (pos0 + v0*t + 1/2*a0*t^2);
    elseif direction == 2
        y = Bodies(i).r(direction);
        fun(funCount,1) = y - (pos0 + v0*t + 1/2*a0*t^2);
    elseif direction == 3
        z = Bodies(i).r(direction);
        fun(funCount,1) = z - (pos0 + v0*t + 1/2*a0*t^2);
    elseif direction >= 4
        if strcmp(ang,'Deg') == 1
            ang0 = deg2rad(pos0);
            w0 = deg2rad(v0);
            alpha0 = deg2rad(a0);
        elseif strcmp(ang,'Rad') == 1
            ang0 = pos0;
            w0 = v0;
            alpha0 = a0;
        end
        rotaxis = Driver(jointCount).rotaxis;
        phimag = ang0 + w0*t + 1/2*alpha0*t^2; %Where pos0 = theta0, v0 = w0, a0 = alpha0
        if rotaxis(1) ~=0
            e1 = Bodies(i).p(direction-3+1);
            phix = rotaxis(1)*phimag;
            fun(funCount,1) = e1 - sin(phix/2);
        elseif rotaxis(2) ~= 0
            e2 = Bodies(i).p(direction-3+1);
            phiy = rotaxis(2)*phimag;
            fun(funCount,1) = e2 - sin(phiy/2);
        elseif rotaxis(3) ~= 0
            e3 = Bodies(i).p(direction-3+1);
            phiz = rotaxis(3)*phimag;
            fun(funCount,1) = e3 - sin(phiz/2);
        end
    end
end

% Form the Jacobian Matrix
if (Flags.Jacobian == 1)
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount,i1:i2)=[0,0,0,0,0,0,0];
    Jacobian(funCount,i1+direction)=1;
end

% Form the r.h.s velocity equations
if(Flags.Velocity == 1)
    niu(funCount) = v0+a0*t;
end

% Form the r.h.s. acceleration equations
if(Flags.Acceleration == 1)
    gamma(funCount) = a0;
end
   
% Update the line counter
funCount = funCount+1;
end