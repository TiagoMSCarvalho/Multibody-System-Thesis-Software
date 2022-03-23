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
%    - Ground: all the information of the Ground constraints present in
%    the multiBodies system
%    - Flags: 4 flags that determine which values to be calculated (each
%    flag for one of the 4 first inputs). 1 means to calculate
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

function [fun,Jacobian,niu,gamma,funCount] = Ground_Constraints(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Ground,Flags)
i = Ground(jointCount).Body;
r = Impose_Column(Bodies(i).r);
p = Impose_Column(Bodies(i).p);
r0 = Impose_Column(Ground(jointCount).r0);      
p0 = Impose_Column(Ground(jointCount).p0);
% Form the position constraint equations
if( Flags.Position == 1)
    fun(funCount:funCount+2,1) = r - r0; % r = r0 and passes to the other side r-r0 = 0;
    fun(funCount+3:funCount+6,1) = p - p0; % talvez falte o e0 = 1;
end

% Form the Jacobian Matrix -> Meter x,y,z,rotx,roty,rotz = 0 para o ground.
if (Flags.Jacobian == 1) && (Flags.Dynamic == 0)
    i1 = 7*(i-1)+1;
    Jacobian(funCount:funCount+6,i1:i1+6) = eye(7);
end

% Form the r.h.s velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+6) = 0;
end

% Form the r.h.s. acceleration equations
if(Flags.Acceleration == 1)
    gamma(funCount:funCount+6) = 0;
end
   
% Update the line counter
funCount = funCount+7; % 7th equation must taken out of the Euler Parameter not from this function.
end