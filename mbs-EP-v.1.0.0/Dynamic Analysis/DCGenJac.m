function [DCJac] = DCGenJac(NBodies,Bodies,Joints)
%This function retrieves the Jacobian but for the generalized coordinates
%(Euler Parameters) that is needed to apply the variables correction to the
%Position Problem.

    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    funCount = 1;
    Jacobian = [];
    
    % Assembly of the Jacobian.
    % For the Ground Constraints
    for jointCount=1:Joints.NGround
        [~,Jacobian,~,~,funCount] = Ground_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Ground,Flags);
    end
    % For the Spherical joints
    for jointCount=1:Joints.NSpherical
        [~,Jacobian,~,~,funCount] = Joint_Spherical([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
    end
    % For the Composite Spherical Joint (SPH - SPH)
    for jointCount=1:Joints.NCompSpherical
        [~,Jacobian,~,~,funCount] = Joint_CompSpherical([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
    end
    % For the Universal joints
    for jointCount=1:Joints.NUniversal
        [~,Jacobian,~,~,funCount] = Joint_Universal([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Universal,Flags); 
    end
    % For the Revolute joints
    for jointCount=1:Joints.NRevolute
        [~,Jacobian,~,~,funCount] = Joint_Revolute([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Revolute,Flags);
    end
    % For the Cylindrical joints
    for jointCount=1:Joints.NCylindrical
        [~,Jacobian,~,~,funCount] = Joint_Cylindrical([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
    end 
    % For the Translation joints
    for jointCount=1:Joints.NTranslation
        [~,Jacobian,~,~,funCount] = Joint_Translation([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Translation,Flags);
    end 
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [~,Jacobian,~,~,funCount] = Simple_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    % For the Euler Parameter Constraint
    for NBod = 2:(NBodies) %takes the first body, ground out of the equation
        [~,Jacobian,~,~,funCount] = EulerParameterConstraint([],Jacobian,[],[],funCount,NBod,Bodies,Flags);
    end
    
    DCJac = Jacobian;

end

