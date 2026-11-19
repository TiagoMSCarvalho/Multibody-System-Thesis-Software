function [fun,Jacobian,niu,gamma,funCount] = EulerParameterConstraint(fun,Jacobian,niu,gamma,funCount,NBod,Bodies,Flags)
%When using the Euler Parameters (e0,e1,e2,e3) there is 7 degrees of
%freedom (7 coordinates) instead of 6 (Bryant Angles), of this 7 only 6
%dofs are fully indenpendt and as a result each body must have a Euler
%Parameter Constraint to relate all 4 Euler Parameters.
%Function with the implementation of the Euler Parameter COnstraint

%fix the number of the Body
i = NBod;

%Allocate the Euler Parameter Vector to a Var
p = Impose_Column(Bodies(i).p);

%State the Constrain Eq: C = P'i*Pi -1 = 0
    %Fsolve already equates this to 0;
%Position Formulation:    
if (Flags.Position == 1)
    fun(funCount,1) = p'*p - 1;
end

%Jacobian Formulation:
if (Flags.Jacobian == 1)
    il = 7*(i-1)+1;
    i2 = il+6;
    Jacobian(funCount,il:i2) = [0,0,0,2*p'];
end

%r.h.s velocity equation
if (Flags.Velocity == 1)
    niu(funCount,1) = 0;
end

%r.h.s acceleration
if (Flags.Acceleration == 1)
    %Isolate the pd from the rest of the qd vector allocated at the Bodie
    %Struct
    pd = Impose_Column(Bodies(i).pd);
    
    gamma(funCount,1) = -pd'*pd; %Alterado o 2 pd'*pd
end

%Jacobian Formulation:
if (Flags.Eqmotion == 1)
    il = 8*(i-1)+1;
    i2 = il+6;
    Jacobian(funCount,il:i2) = [0,0,0,2*p'];
end
    
%Update the function counter
funCount = funCount+1;
end

