function [Bodies] = DynDriverVel(Bodies,Driver,jointCount,time,driverfunctions)
%% Pre-processing Variables
    i = Driver(jointCount).Body;  
    direction = Driver(jointCount).direction;
    functype = driverfunctions(jointCount).Type;
    inputfunc = driverfunctions(jointCount).functions;
    inputfunc = str2func(inputfunc);
    syms t ; %Creates the symbolic variable t to allow the functioning of the diff function;
%% Calculus of the w and rd value
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
        rd = zeros(3,1);
        if direction == 1
            rd(1,1) = funcvalue;
        elseif direction == 2
            rd(2,1) = funcvalue;
        elseif direction == 3
            rd(3,1) = funcvalue;
        end
        Bodies(i).rd = Bodies(i).rd + rd;
        
    elseif direction > 3
        w = zeros(3,1);
        if direction == 4
            w(1,1) = funcvalue;
        elseif direction == 5
            w(2,1) = funcvalue;
        elseif direction == 6
            w(3,1) = funcvalue;
        end
        Bodies(i).w = Bodies(i).w + w;
    end
end

