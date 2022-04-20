function [qc,vc,Bodies] = RKDirectCorrection(y,NBodies,Bodies,Joints,SimType,driverfunctions,tf)
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
%% phi(qu) Calculation
     % Non Linear Opts Solver
     opts=optimoptions('fsolve');
     opts.Algorithm='levenberg-marquardt';
     opts.Diagnostics = 'off';
     opts.Display = 'iter-detailed';
     opts.ScaleProblem = 'Jacobian';
     opts.UseParallel = false;
     opts.FunctionTolerance = 1e-4;
     opts.StepTolerance = 1e-4;
     opts.OptimalityTolerance = 1e-4;
     % Flags definition
     Flags.Position = 1;
     Flags.Jacobian = 1;
     Flags.Velocity = 0;
     Flags.Acceleration = 0;
     Flags.Dynamic = 0;
     Flags.AccelDyn = 0;
     %Update of the body postures
     Bodies = UpdateBodyPostures(qu,NBodies,Bodies);
     %Calculus of the phi(qu)
     dcconstrains = @(qc)RKposdc(Joints,Bodies,NBodies,Flags,qu,driverfunctions,tf); %t0 -> tf;
     [qc] = fsolve(dcconstrains,qu,opts); 
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
     %Update Velocities
     Bodies = UpdateVelocities(vu,NBodies,Bodies,SimType);
     %Calculus of the phid(qc,vu)
     [phid] = RKveldc(Joints,Bodies,Flags,driverfunctions,tf);
     % vu correction
     vc = vu - pinv(phid)*(phid*vu);
     %Update of body velocities with the correction for t + timestep (vc)
     Bodies = UpdateVelocities(vc,NBodies,Bodies,SimType);
     
     
end

