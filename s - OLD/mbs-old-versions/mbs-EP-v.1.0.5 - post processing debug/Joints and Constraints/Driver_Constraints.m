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

% Overall Theory:
% Page 100 of Nikravesh
% The Driving Links equations are of the form:
%      fun(funcount,1) = xi - d(t) = 0
%      fun(funcount,2) = yi - d(t) = 0


function [fun,Jacobian,Ct,Ctt,funCount] = Driver_Constraints(fun,Jacobian,Ct,Ctt,funCount,jointCount, Bodies, Driver,Flags,time,driverfunctions)
%% Pre-processing Variables
i = Driver(jointCount).Body;  
% Vectors and Direction
direction = Driver(jointCount).direction;
vector = Driver(jointCount).axis;
[~,vectordir] = unitvector(vector);
% Functions of Disp/Vel and Acceleration
dispfunc = str2func(driverfunctions(jointCount).dispfunc);
velfunc = str2func(driverfunctions(jointCount).velfunc);
accelfunc = str2func(driverfunctions(jointCount).accelfunc);


%% Form the position constraint equations
if( Flags.Position == 1)
    if direction < 4
       %Displacement Magnitude and Vector
       dispvalue = dispfunc(time);
       dispvector = dispvalue*vectordir;
       
       %Chooses between the body r and point P (Not Implemented Yet)
       r = Bodies(i).r;
       
       %Allocation of the constraints
       j = 0;
       if vectordir(1) ~=0
           fun(funCount+j,1) = r(1) - dispvector(1);
           j = j + 1;
       end
       if vectordir(2) ~=0
           fun(funCount+j,1) = r(2) - dispvector(2);
           j = j + 1;
       end
       if vectordir(3) ~=0
           fun(funCount+j,1) = r(3) - dispvector(3);
           j = j + 1;
       end
      
    elseif direction >= 4
        % Angular Displacement Magnitude and Vector
        phimag = dispfunc(time);
        rotvector = phimag*vectordir;
        
        %Body Angular Vector
        p = Bodies(i).p;

        %Allocation of the constraints
        j= 0;
        if vectordir(1) ~=0
            fun(funCount+j,1) = p(2) - sin(rotvector(1)/2);
            j = j + 1;
        end
        if vectordir(2) ~=0
            fun(funCount+j,1) = p(3) - sin(rotvector(2)/2);
            j = j + 1;
        end
        if vectordir(3) ~=0
            fun(funCount+j,1) = p(4) - sin(rotvector(3)/2);
            j = j + 1;
        end

    end
end

%% Form the Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 0)
    i1 = 7*(i-1)+1;
    j = 0;
    %Translation
    if direction < 4
        if vectordir(1) ~= 0 
            Jacobian(funCount+j,i1) = 1;
            j = j + 1;
        end
        if vectordir(2) ~= 0
            Jacobian(funCount+j,i1+1) = 1;
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Jacobian(funCount+j,i1+2) = 1;
            j = j + 1;
        end
    %Rotation        
    elseif direction > 3
        if vectordir(1) ~= 0
            Jacobian(funCount+j,i1+4) = 1;
            j = j + 1;
        end
        if vectordir(2) ~= 0
            Jacobian(funCount+j,i1+5) = 1;
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Jacobian(funCount+j,i1+6) = 1;
            j = j + 1;
        end
    end
end

%% Form the r.h.s velocity equations
if(Flags.Velocity == 1) 
    % Velocity Vector and Magnitude
    velvalue = velfunc(time);
    velvector = Impose_Column(velvalue*vectordir);
    j = 0;

    if direction < 4
        if vectordir(1) ~= 0
            Ct(funCount+j,1) = velvector(1);
            j = j + 1;
        end
        if vectordir(2) ~= 0
            Ct(funCount+j,1) = velvector(2);
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Ct(funCount+j,1) = velvector(3);
            j = j + 1;
        end
    elseif direction > 3    
        G = Bodies(i).G;
        pd = (1/2)*(G.'*velvector);
        %Allocation of p0;
        i1 = 7*(i-1)+1;
        Ct(i1+3,1) = pd(1);
        if vectordir(1) ~= 0
            Ct(funCount+j,1) = pd(2);
            j = j +1;
        end
        if vectordir(2) ~= 0
            Ct(funCount+j,1) = pd(3);
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Ct(funCount+j,1) = pd(4);
            j = j + 1;
        end
    end
end

%% Form the r.h.s. acceleration equations
if(Flags.Acceleration == 1)
    %Velocity Vector and Magnitude
    velvalue = velfunc(time);
    velvector = Impose_Column(velvalue*vectordir);
    %Aceleration Vector and Magnitude
    accelvalue = accelfunc(time);
    accelvector = Impose_Column(accelvalue*vectordir);
    j = 0;
    
    if direction <4
        if vectordir(1) ~= 0
            Ctt(funCount+j,1) = accelvector(1);
            j = j + 1;
        end
        if vectordir(2) ~= 0
            Ctt(funCount+j,1) = accelvector(2);
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Ctt(funCount+j,1) = accelvector(3);
            j = j +1;
        end
    elseif direction > 3
        p = Bodies(i).p;
        G = Bodies(i).G;
        pdd = (1/2)*(G.'*accelvector) + (1/4)*(velvector'*velvector)*p; 
        %This is what changes the value from -4.9 to -3.3 in accel ( it was
        %changed for the dynamics be careful). Kin: + (1/4)
        i1 = 7*(i-1)+1;
        Ctt(i1+3,1) = pdd(1);
        if vectordir(1) ~= 0
           Ctt(funCount+j,1) = pdd(2);
           j = j + 1;
        end
        if vectordir(2) ~= 0
            Ctt(funCount+j,1) = pdd(3);
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Ctt(funCount+j,1) = pdd(4);
            j = j + 1;
        end
    end
end

%% Joint Formulation - Dynamic Problem
% Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 1)
    i1 = 6*(i-1)+1;
    j = 0;
    %Translation
    if direction < 4
        if vectordir(1) ~= 0 
            Jacobian(funCount+j,i1) = 1;
            j = j + 1;
        end
        if vectordir(2) ~= 0
            Jacobian(funCount+j,i1+1) = 1;
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Jacobian(funCount+j,i1+2) = 1;
            j = j + 1;
        end
    %Rotation        
    elseif direction > 3
        if vectordir(1) ~= 0
            Jacobian(funCount+j,i1+3) = 1;
            j = j + 1;
        end
        if vectordir(2) ~= 0
            Jacobian(funCount+j,i1+4) = 1;
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Jacobian(funCount+j,i1+5) = 1;
            j = j + 1;
        end
    end
end
%mesmo problema que tinha anteriormente
if(Flags.AccelDyn == 1) && (time ~= 0)
    %Aceleration Vector and Magnitude
    accelvalue = accelfunc(time);
    accelvector = Impose_Column(accelvalue*vectordir);
    j = 0;
    
    if direction < 4
       if vectordir(1) ~= 0
           Ctt(funCount+j,1) = accelvector(1);
           j = j +1;
       end
       if vectordir(2) ~= 0
           Ctt(funCount+j,1) = accelvector(2);
           j = j + 1;
       end
       if vectordir(3) ~= 0
           Ctt(funCount+j,1) = accelvector(3);
           j = j + 1;
       end
              
    elseif direction > 3
        if vectordir(1) ~= 0
            Ctt(funCount+j,1) = accelvector(1);
            j = j + 1;
        end
        if vectordir(2) ~= 0 
            Ctt(funCount+j,1) = accelvector(2);
            j = j + 1;
        end
        if vectordir(3) ~= 0
            Ctt(funCount+j,1) = accelvector(3);
            j = j + 1;
        end        
    end
end
   
%% Update the line counter
funCount = funCount+j;
end