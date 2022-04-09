function [qc,vc,Bodies] = RKDirectCorrection(y,NBodies,Bodies,Jacobian,DCJac,Joints,SimType)
%This function implements the Direct Correction stated in the Constraint Violation
    % Equations that are implemented:
        % qc = qu - D'*inv(D*D') * phi(qu);
        % vc = vu - D'*inv(D*D') * phid(qc,vu);
            %phid = D*vu;
    
     % qu and vu vector allocation
     qu = y(1:7*NBodies,1);
     i1 = 7*NBodies + 1;
     i2 = 7*NBodies + 6*NBodies;
     vu = y(i1:i2,1);
     D = Jacobian; %D is the Jacobian written with the initial pos and vel

%% phi(qu) Calculation
     % Flags definition
     Flags.Position = 1;
     Flags.Jacobian = 0;
     Flags.Velocity = 0;
     Flags.Acceleration = 0;
     Flags.Dynamic = 0;
     Flags.AccelDyn = 0;
     %Update of the body postures
     Bodies = UpdateBodyPostures(qu,NBodies,Bodies);
     %Calculus of the phi(qu)
     [phi,~] = RKDCJoints(Joints,Bodies,NBodies,Flags);
     % qu correction
     qc = qu - DCJac'*inv(DCJac*DCJac')*phi;
     %Update of the body postures with the correction for t + timestep (qc)
     Bodies = UpdateBodyPostures(qc,NBodies,Bodies);
     
%% phid (qc,vu) Calculation
     %Flags definition
     Flags.Position = 0;
     Flags.Jacobian = 1;
     Flags.Velocity = 0;
     Flags.Acceleration = 0;
     Flags.Dynamic = 1;
     Flags.AccelDyn = 0;
     %Calculus of the phid(qc,vu)
     [~,phid] = RKDCJoints(Joints,Bodies,[],Flags);
     % vu correction
     vc = vu - D'*inv(D*D')*(phid*vu);
     %Update of body velocities with the correction for t + timestep (vc)
     Bodies = UpdateVelocities(vc,NBodies,Bodies,SimType);
     
     
end

