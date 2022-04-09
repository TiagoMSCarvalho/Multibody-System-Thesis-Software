function [phi,phid] = RKDCJoints(Joints,Bodies,NBodies,Flags)
%Joins and calculates the joints equations for the velocity and position
%Direct Correction. (Avoids Redudant Code at RKDirectCorrection)

phi = [];
phid = [];
funCount=1;

    if Flags.Position == 1     
        fun = [];
        % For the Ground Constraints
        for jointCount=1:Joints.NGround
            [fun,~,~,~,funCount] = Ground_Constraints(fun,[],[],[],funCount,jointCount, Bodies, Joints.Ground,Flags);
        end
        % For the Spherical Joints
        for jointCount=1:Joints.NSpherical
            [fun,~,~,~,funCount] = Joint_Spherical(fun,[],[],[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
        end
        % For the Composite Spherical Joint (SPH - SPH)
        for jointCount=1:Joints.NCompSpherical
            [fun,~,~,~,funCount] = Joint_CompSpherical(fun,[],[],[],funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
        end
        % For the Universal joints
        for jointCount=1:Joints.NUniversal
            [fun,~,~,~,funCount] = Joint_Universal(fun,[],[],[],funCount,jointCount, Bodies, Joints.Universal,Flags); 
        end
        % Form the Revolute joints
        for jointCount=1:Joints.NRevolute
            [fun,~,~,~,funCount] = Joint_Revolute(fun,[],[],[],funCount,jointCount, Bodies, Joints.Revolute,Flags);
        end
        % For the Cylindrical joints
        for jointCount=1:Joints.NCylindrical
            [fun,~,~,~,funCount] = Joint_Cylindrical(fun,[],[],[],funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
        end
        % For the Translation joints
        for jointCount=1:Joints.NTranslation
            [fun,~,~,~,funCount] = Joint_Translation(fun,[],[],[],funCount,jointCount, Bodies, Joints.Translation,Flags);
        end
        % For the Simple Constraints
        for jointCount=1:Joints.NSimple
            [fun,~,~,~,funCount] = Simple_Constraints(fun,[],[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
        end
        %Euler Parameter Constraints
        for NBod = 2:(NBodies) %takes the first body, ground out of the equation
            [fun,~,~,~,funCount] = EulerParameterConstraint(fun,[],[],[],funCount,NBod,Bodies,Flags);
        end
        
        phi = fun; %Variable allocation
        
    elseif Flags.Jacobian == 1 && Flags.Dynamic == 1
        Jacobian = [];
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
       
        phid = Jacobian; %variable allocation
        
    end
end

