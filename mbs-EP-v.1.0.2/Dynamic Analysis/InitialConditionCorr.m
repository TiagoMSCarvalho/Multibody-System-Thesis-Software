function [Bodies] = InitialConditionCorr(NBodies,Bodies,Joints,SimType,driverfunctions)
%Function used for the debug of the initial conditions problem.    
%% Correction    
deltamax = 1;
while deltamax > 1e-3

    %Pre Setup of the variables
    qi = CreateAuxiliaryBodyStructure(NBodies,Bodies);
    Bodies = DynCalcAGL(qi,NBodies,Bodies);

    % Flags Position
    Flags.Position = 1;
    Flags.Jacobian = 0;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 0;
    Flags.AccelDyn = 0;
    funCount = 1;
    fun = [];

    %% Position    
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
    % For the Spherical Revolute joints
    for jointCount=1:Joints.NSphRev
        [fun,~,~,~,funCount] = Joint_CompSphRev(fun,[],[],[],funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
    end
    % For the Translation Revolute Composite joints
    for jointCount=1:Joints.NTraRev
        [fun,~,~,~,funCount] = Joint_CompTraRev(fun,[],[],[],funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
    end
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [fun,~,~,~,funCount] = Simple_Constraints(fun,[],[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    %Euler Parameter Constraints
    for NBod = 2:NBodies %takes the first body, ground out of the equation
        [fun,~,~,~,funCount] = EulerParameterConstraint(fun,[],[],[],funCount,NBod,Bodies,Flags);
    end
    % For the Driver Constraints
    for jointCount=1:Joints.NDriver
        [fun,~,~,~,funCount] = Driver_Constraints(fun,[],[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,0,driverfunctions);
    end
    %% Jacobian
        Flags.Position = 0;
        Flags.Jacobian = 1;
        Flags.Velocity = 1;
        Flags.Acceleration = 0;
        Flags.Dynamic = 0;
        Flags.AccelDyn = 0;
        funCount = 1;
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
    % For the Spherical Revolute joints
    for jointCount=1:Joints.NSphRev
        [~,Jacobian,~,~,funCount] = Joint_CompSphRev([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
    end 
    % For the Translation Revolute Composite joints
    for jointCount=1:Joints.NTraRev
        [~,Jacobian,~,~,funCount] = Joint_CompTraRev([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
    end 
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [~,Jacobian,~,~,funCount] = Simple_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    % For the Euler Parameter Constraint
    for NBod = 2:(NBodies) %takes the first body, ground out of the equation
        [~,Jacobian,~,~,funCount] = EulerParameterConstraint([],Jacobian,[],[],funCount,NBod,Bodies,Flags);
    end
    % For the Driver Constraints
    for jointCount=1:Joints.NDriver
        [~,Jacobian,~,~,funCount] = Driver_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,0,driverfunctions);
    end

    %% Augmented Mass Matrix Assembly
    % Mass Matrix
    massmatrix = zeros(6*NBodies,6*NBodies); %Pre-Allocation
    for i = 1:NBodies
        Mass = Bodies(i).Mass;
        B = Bodies(i).L;
        Inertia = Bodies(i).Inertia;
        Irat = 4*B'*diag(Inertia)*B; %Nikra Article on the use of EP for 3D Dynamics
        i1 = 7*(i-1)+1;
        massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
        massmatrix(i1+3:i1+6,i1+3:i1+6) = Irat; 
    end
    
    % fun is omega
    
    deltaq = -pinv(Jacobian)*fun;
    q = qi + deltaq;
    
    delta = q - qi;
    deltamax = max(abs(delta));

    [Bodies] = UpdateBodyPostures(q,NBodies,Bodies);
    
end

%% Velocity Correction

    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 1;
    Flags.AccelDyn = 0;
    funCount = 1;
    Jacobian = [];
    
    %% Augmented Mass Matrix Assembly
    % Mass Matrix
    massmatrix = zeros(6*NBodies,6*NBodies); %Pre-Allocation
    for i = 1:NBodies
        Mass = Bodies(i).Mass;
        %A = Bodies(i).A;
        Inertia = Bodies(i).Inertia;
        %Irat = A*diag(Inertia)*A'; %Inertia convertion to Global Inertia Tensor (Nikra) - Rotated Theorem (Paulo Flores)
        i1 = 6*(i-1)+1;
        massmatrix(i1:i1+2,i1:i1+2) = Mass * eye(3);
        massmatrix(i1+3:i1+5,i1+3:i1+5) = diag(Inertia);
    end
    
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
    % For the Spherical Revolute Joints
    for jointCount=1:Joints.NSphRev
        [~,Jacobian,~,~,funCount] = Joint_CompSphRev([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
    end
    %For the Translational - Revolute joints
    for jointCount=1:Joints.NTraRev
        [~,Jacobian,~,~,funCount] = Joint_CompTraRev([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
    end
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [~,Jacobian,~,~,funCount] = Simple_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
    end
    % For the Driving Constraints
    for jointCount=1:Joints.NDriver
        [~,Jacobian,~,~,funCount] = Driver_Constraints([],Jacobian,[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,0,driverfunctions); 
    end

    for i = 1:NBodies
       i1 = 6*(i-1)+1;
       v0(i1:i1+5,1) = [Impose_Column(Bodies(i).rd);Impose_Column(Bodies(i).w)];
    end
    
    epsilon = Jacobian*v0;
    
    deltav = -pinv(Jacobian)*epsilon;
    
    v = v0 + deltav;
    
    [Bodies] = UpdateVelocities(v,NBodies,Bodies,SimType);

end

