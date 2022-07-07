function [qc] = DCPos(Joints,Bodies,NBodies,Flags,qu,driverfunctions,t)
%Calculates the position constrains system, and set ups the system to be
%solved by the lavanberg-marquardt algorithm.
%Direct Correction is now seperated due to the fact that the position
%correction needs to be solved iteratively.

count = 0;
deltamax = 5;

%while deltamax > 1*10^-3
    funCount=1;
    fun = [];
    Jacobian = [];
    qc = [];
    % For the Ground Constraints
    for jointCount=1:Joints.NGround
        [fun,Jacobian,~,~,funCount] = Ground_Constraints(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Ground,Flags);
    end
    % For the Spherical Joints
    for jointCount=1:Joints.NSpherical
        [fun,Jacobian,~,~,funCount] = Joint_Spherical(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
    end
    % For the Composite Spherical Joint (SPH - SPH)
    for jointCount=1:Joints.NCompSpherical
        [fun,Jacobian,~,~,funCount] = Joint_CompSpherical(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
    end
    % For the Universal joints
    for jointCount=1:Joints.NUniversal
        [fun,Jacobian,~,~,funCount] = Joint_Universal(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Universal,Flags); 
    end
    % Form the Revolute joints
    for jointCount=1:Joints.NRevolute
        [fun,Jacobian,~,~,funCount] = Joint_Revolute(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Revolute,Flags);
    end
    % For the Cylindrical joints
    for jointCount=1:Joints.NCylindrical
        [fun,Jacobian,~,~,funCount] = Joint_Cylindrical(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
    end
    % For the Translation joints
    for jointCount=1:Joints.NTranslation
        [fun,Jacobian,~,~,funCount] = Joint_Translation(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Translation,Flags);
    end
    % For the Spherical Revolute joints
    for jointCount=1:Joints.NSphRev
        [fun,Jacobian,~,~,funCount] = Joint_CompSphRev(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
    end
    % For the Tra Rev Composite joint
    for jointCount=1:Joints.NTraRev
        [fun,Jacobian,~,~,funCount] = Joint_CompTraRev(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
    end
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [fun,Jacobian,~,~,funCount] = Simple_Constraints(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    %Euler Parameter Constraints
    for NBod = 2:(NBodies) %takes the first body, ground out of the equation
        [fun,Jacobian,~,~,funCount] = EulerParameterConstraint(fun,Jacobian,[],[],funCount,NBod,Bodies,Flags);
    end
    % For the Driver Constraints
    for jointCount=1:Joints.NDriver
        [fun,Jacobian,~,~,funCount] = Driver_Constraints(fun,Jacobian,[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,t,driverfunctions);
    end

    deltaq = -pinv(Jacobian,1e-6)*fun; % Paulo Flores: -D'*(D*D')^-1*fun

    qc = qu + deltaq;
    
    delta = qc - qu;
    deltamax = max(abs(delta));
    
    Bodies = UpdateBodyPostures(qc,NBodies,Bodies);
    
    %Setup new qu vector
    qu = CreateAuxiliaryBodyStructure(NBodies,Bodies);

    count = count+1;
%end

end