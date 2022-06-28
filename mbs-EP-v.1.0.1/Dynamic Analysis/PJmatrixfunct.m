function [fun,Jacobian,niu,gamma] = PJmatrixfunct(Flags,Bodies,NBodies,Joints,debugdata,driverfunctions,coord)
%% Pré-setup of the variables
    funCount = 1;
    fun = [];
    Jacobian = [];
    niu = zeros(debugdata(1).cdof,1);
    gamma = zeros(debugdata(1).cdof,1);
    Ct = zeros(debugdata(1).cdof,1);
    Ctt = zeros(debugdata(1).cdof,1);

%% Assembly of the Position,Jacobian matrix and rhs vel or accel.
    % For the Ground Constraints
    for jointCount=1:Joints.NGround
        [fun,Jacobian,niu,gamma,funCount] = Ground_Constraints(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.Ground,Flags);
    end
    % For the Spherical joints
    for jointCount=1:Joints.NSpherical
        [fun,Jacobian,niu,gamma,funCount] = Joint_Spherical(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.Spherical,Flags);
    end
    % For the Composite Spherical Joint (SPH - SPH)
    for jointCount=1:Joints.NCompSpherical
        [fun,Jacobian,niu,gamma,funCount] = Joint_CompSpherical(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
    end
    % For the Universal joints
    for jointCount=1:Joints.NUniversal
        [fun,Jacobian,niu,gamma,funCount] = Joint_Universal(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.Universal,Flags); 
    end
    % For the Revolute joints
    for jointCount=1:Joints.NRevolute
        [fun,Jacobian,niu,gamma,funCount] = Joint_Revolute(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.Revolute,Flags);
    end
    % For the Cylindrical joints
    for jointCount=1:Joints.NCylindrical
        [fun,Jacobian,niu,gamma,funCount] = Joint_Cylindrical(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
    end 
    % For the Translation joints
    for jointCount=1:Joints.NTranslation
        [fun,Jacobian,niu,gamma,funCount] = Joint_Translation(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.Translation,Flags);
    end
    % For the Spherical Revolute Joints
    for jointCount=1:Joints.NSphRev
        [fun,Jacobian,niu,gamma,funCount] = Joint_CompSphRev(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
    end
    %For the Translational - Revolute joints
    for jointCount=1:Joints.NTraRev
        [fun,Jacobian,niu,gamma,funCount] = Joint_CompTraRev(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
    end
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [fun,Jacobian,niu,gamma,funCount] = Simple_Constraints(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    %Euler Parameter Constraints
    if coord == 7
        for NBod = 2:NBodies %takes the first body, ground out of the equation
            [fun,Jacobian,niu,gamma,funCount] = EulerParameterConstraint(fun,Jacobian,niu,gamma,funCount,NBod,Bodies,Flags);
        end
    end
    % For the Driving Constraints
    for jointCount=1:Joints.NDriver
        [fun,Jacobian,niu,Ctt,funCount] = Driver_Constraints(fun,Jacobian,Ct,Ctt,funCount,jointCount, Bodies, Joints.Driver,Flags,time,driverfunctions); 
    end
    
    %% Assembly of the rhs vectors, if they are needed.
    niu = niu - Ct; %rhs velocity
    gamma = gamma - Ctt; % rhs acceleration

end

