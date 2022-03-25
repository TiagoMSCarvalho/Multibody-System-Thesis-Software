function [outputArg1,outputArg2] = DynInitialAccel(Joints,NBodies,Bodies,t,it)
%This function uses the inputs of initial position, initial velocities and
%forces to calculate the initial acceleration that will be fed to the
%Runge-Kutta ODE45 solver.

%% Set Up of the variables needed to construct the Matrix
% Pre Allocation of the q0 vector
    q0 = CreateAuxiliaryBodyStructure(NBodies,Bodies);
% Calculation of the Matrixes A, G and L
    Bodies = DynCalcAGL(q0,NBodies,Bodies);
% Allocation of the Flags Struct
    % Flags.Dynamic, is a new field that was introduced to help to separate
    % the Kinematic Program from the Dynamic Program, and to keep the
    % program modular and of easy debug.
    Flags.Position = 0;
    Flags.Jacobian = 1;
    Flags.Velocity = 0;
    Flags.Acceleration = 0;
    Flags.Dynamic = 1;
    Flags.AccelDyn = 1;
%% Calculation of the Kinematic Converted Jacobian
    funCount = 1;
    % For the Ground Constraints
    for jointCount=1:Joints.NGround
        [~,Jacobian,~,~,funCount] = Ground_Constraints([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Ground,Flags);
    % For the Spherical joints
    for jointCount=1:Joints.NSpherical
        [~,Jacobian,~,~,funCount] = Joint_Spherical([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
    end
    % For the Composite Spherical Joint (SPH - SPH)
    for jointCount=1:Joints.NCompSpherical
        [~,Jacobian,~,~,funCount] = Joint_CompSpherical([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
    end
    % For the Universal joints
    for jointCount=1:Joints.NUniversal
        [~,Jacobian,~,~,funCount] = Joint_Universal([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Universal,Flags); 
    end
    % For the Revolute joints
    for jointCount=1:Joints.NRevolute
        [~,Jacobian,~,~,funCount] = Joint_Revolute([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Revolute,Flags);
    end
    % For the Cylindrical joints
    for jointCount=1:Joints.NCylindrical
        [~,Jacobian,~,~,funCount] = Joint_Cylindrical([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
    end 
    % For the Translation joints
    for jointCount=1:Joints.NTranslation
        [~,Jacobian,~,~,funCount] = Joint_Translation([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Translation,Flags);
    end 
    % For the Simple Constraints
    for jointCount=1:Joints.NSimple
        [~,Jacobian,~,~,funCount] = Simple_Constraints([],Jacobian,niu,[],funCount,jointCount, Bodies, Joints.Simple,Flags);
    end

%% Function Responsible for the Force Vectors    
    % Atribuir Forças e Momentos ao vetor g, estes estão guardados na
    % struct Bodies e chamar a struct dos elementos força, assim como
    % calcular de cada um como fosse uma joint.
end

