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


function [fun,Jacobian,niu,gamma,funCount] = Driver_Constraints(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Driver,Flags,t)
i = Driver(jointCount).Body;  
direction = Driver(jointCount).direction;      
pos0 = Driver(jointCount).pos0;
v0 = Driver(jointCount).v0; 
a0 = Driver(jointCount).a0;

if  direction < 4
    pos = Bodies(i).r(direction);
else
    pos = Bodies(i).p(direction-3);
end
% Form the position constraint equations
if( Flags.Position == 1)
    fun(funCount,1) = pos + pos0 + v0*t + 1/2*a0*t^2;
end

% Form the Jacobian Matrix
if (Flags.Jacobian == 1)
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount,i1:i2)=[0,0,0,0,0,0,0];
    Jacobian(funCount,i1+direction-1)=1;
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