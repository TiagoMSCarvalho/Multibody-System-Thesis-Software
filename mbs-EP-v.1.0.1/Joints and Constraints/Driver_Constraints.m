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


function [fun,Jacobian,Ct,Ctt,funCount] = Driver_Constraints(fun,Jacobian,Ct,Ctt,funCount,jointCount, Bodies, Driver,Flags,time,driverfunctions)
%% Pre-processing Variables
i = Driver(jointCount).Body;  
direction = Driver(jointCount).direction;
functype = driverfunctions(jointCount).Type;
inputfunc = driverfunctions(jointCount).functions;
%This piece of codes is what enables to have constant drivers inputs.
if strcmp(functype,"Numeric") == 1 || strcmp(functype,"numeric") == 1
    inputfunc = str2double(inputfunc{1,1});
elseif ischar(inputfunc{1,1})
    inputfunc = convertCharsToStrings(inputfunc{1,1});
    inputfunc = str2func(inputfunc);
end
%Vector may not be unitary, needs to be recalculated, so the unitvector function is called.
rotaxis = unitvector(Driver(jointCount).rotaxis);
vectordir = unitvector(Driver(jointCount).rotaxis);
syms t ; %Creates the symbolic variable t to allow the functioning of the diff function;

%It is needed to remake for the numeric case ( a constant).


%% Form the position constraint equations
if( Flags.Position == 1)
    r = Bodies(i).r;
    funcvalue = inputfunc(time);
    if direction < 4
        if vectordir(1) ~=0
            x = r(direction);
            fun(funCount,1) = x - funcvalue;
        elseif vectordir(2) ~=0
            y = r(direction);
            fun(funCount,1) = y - funcvalue;
        elseif vectordir(3) ~=0
            z = r(direction);
            fun(funCount,1) = z - funcvalue;
        end
    elseif direction >= 4
        p = Bodies(i).p;
        phimag = funcvalue;
        if rotaxis(1) ~=0
            e1 = p(direction-3+1);
            phix = sin(phimag/2);
            fun(funCount,1) = e1 - phix;
        elseif rotaxis(2) ~= 0
            e2 = p(direction-3+1);
            phiy = sin(phimag/2);
            fun(funCount,1) = fun(funCount,1) + e2 - phiy;
        elseif rotaxis(3) ~= 0
            e3 = p(direction-3+1);
            phiz = sin(phimag/2);
            fun(funCount,1) = fun(funCount,1) + e3 - phiz;
        end
    end
end

%% Form the Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 0)
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount,i1:i2)=[0,0,0,0,0,0,0];
    if direction < 4
        Jacobian(funCount,i1+direction-1) = 1;
    elseif direction > 3
        Jacobian(funCount,i1+direction) = 1;
    end
end

%% Form the r.h.s velocity equations
if(Flags.Velocity == 1) 
    der = diff(inputfunc,t);
    dfuncdt = matlabFunction(der);
    if strcmp(functype,'Sinusoidal') == 1
        degree = 2;
    elseif strcmp(functype,'Polynomial') == 1
        degree = polynomialDegree(der);
    end
    if degree >= 1
        funcvalue = dfuncdt(time);
    elseif degree == 0
        funcvalue = double(der);
    end
    if direction < 4
        Ct(funCount,1) = funcvalue;
    elseif direction > 3
        w = zeros(4,1);
        if direction == 4
            w(2,1) = funcvalue;
        elseif direction == 5
            w(3,1) = funcvalue;
        elseif direction == 6
            w(4,1) = funcvalue;
        end
        p = Bodies(i).p;
        G = Bodies(i).G;
        E = [p(1),p(2),p(3),p(4);G];
        pd = (1/2)*(E.'*w);
        i1 = 7*(i-1)+1;
        Ct(i1+3,1) = pd(1);
        if pd(2) ~=0 && abs(pd(2)) > 0.01
            Ct(funCount,1) = pd(2);
        elseif pd(3) ~= 0 && abs(pd(3)) > 0.01
            Ct(funCount,1) = pd(3);
        elseif pd(4) ~= 0 && abs(pd(4)) > 0.01
            Ct(funCount,1) = pd(4);
        end
    end
end

%% Form the r.h.s. acceleration equations
if(Flags.Acceleration == 1)
    w = zeros(3,1);
    der  = diff(inputfunc,t);
    dfuncdt = matlabFunction(der);
    if strcmp(functype,'Sinusoidal') == 1
        degree = 2;
    elseif strcmp(functype,'Polynomial') == 1
        degree = polynomialDegree(der);
    end
    if degree >= 1
        funcvalue = dfuncdt(time);
    elseif degree == 0
        funcvalue = double(der);
    end
    if direction == 4
       w(1,1) = funcvalue;
    elseif direction == 5
       w(2,1) = funcvalue;
    elseif direction == 6
       w(3,1) = funcvalue;
    end
    dder = diff(der,t);
    ddfuncdt = matlabFunction(dder);
    if strcmp(functype,'Sinusoidal') == 1
        degree = 2;
    elseif strcmp(functype,'Polynomial') == 1
        degree = polynomialDegree(dder);
    end
    if degree >= 1
        funcvalue = ddfuncdt(time);
    elseif degree == 0
        funcvalue = double(dder);
    end
    if direction < 4
        Ctt(funCount,1) = funcvalue;
    elseif direction > 3
        alpha = zeros(3,1);
        if direction == 4
            alpha(1,1) = funcvalue;
        elseif direction == 5
            alpha(2,1) = funcvalue;
        elseif direction == 6
            alpha(3,1) = funcvalue;
        end
        p = Bodies(i).p;
        G = Bodies(i).G;
        pdd = (1/2)*(G.'*alpha) + (1/4)*(w'*w)*p; %This is what changes the value from -4.9 to -3.3 in accel;
        i1 = 7*(i-1)+1;
        Ctt(i1+3,1) = pdd(1);
        if pdd(2) ~=0 && abs(pdd(2)) > 0.0001 
            Ctt(funCount,1) = pdd(2);
        elseif pdd(3) ~= 0 && abs(pdd(3)) > 0.0001
            Ctt(funCount,1) = pdd(3);
        elseif pdd(4) ~= 0 && abs(pdd(4)) > 0.0001
            Ctt(funCount,1) = pdd(4);
        end
    end
end
%% Joint Formulation - Dynamic Problem
% Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 1)
    i1 = 6*(i-1)+1;
    i2  = i1+5;
    Jacobian(funCount,i1:i2)=[0,0,0,0,0,0];
    Jacobian(funCount,i1+direction-1) = 1;
end
%mesmo problema que tinha anteriormente
if(Flags.AccelDyn == 1) && (time ~= 0)
    der  = diff(inputfunc,t);
    dfuncdt = matlabFunction(der);
    if strcmp(functype,'Sinusoidal') == 1
        degree = 2;
    elseif strcmp(functype,'Polynomial') == 1
        degree = polynomialDegree(der);
    end
    if degree >= 1
        funcvalue = dfuncdt(time);
    elseif degree == 0
        funcvalue = double(der);
    end
    dder = diff(der,t);
    ddfuncdt = matlabFunction(dder);
    if strcmp(functype,'Sinusoidal') == 1
        degree = 2;
    elseif strcmp(functype,'Polynomial') == 1
        degree = polynomialDegree(dder);
    end
    if degree >= 1
        funcvalue = ddfuncdt(time);
    elseif degree == 0
        funcvalue = double(dder); %converts input do double
    end
        Ctt(funCount,1) = funcvalue;
end
   
%% Update the line counter
funCount = funCount+1;
end