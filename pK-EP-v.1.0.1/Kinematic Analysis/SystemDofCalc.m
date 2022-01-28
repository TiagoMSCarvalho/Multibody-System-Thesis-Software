function [debugdata] = SystemDofCalc(NBodies,Bodies,Joints,debugdata)
%This function calculates the number of DoFs to ensure that the multibody is properly constrained;
    %This function is also responsible for warning the user for an issue
    %with the contrains to facilitate the problem debug.
    %Keep in mind that for Kinematic Analyis the Jacobian matrix must be
    %squared and the odofs result should be 0, (Wichita Thesis)
    
    bdof = 7; % Euler Parameters imply 7 coord
    cdof = 7; % Global Reference Constraint
    
    
    %Accounts for the spherical dofs
    for i = 1:Joints.NSpherical
        cdof = cdof + 3;
    end
    %Accounts for the universal dofs
    for i = 1:Joints.NUniversal
        cdof = cdof + 4;
    end
    %Accounts for the revolute dofs
    for i = 1:Joints.NRevolute
        cdof = cdof + 5;
    end
    %Accounts for the cylindrical
    for i = 1:Joints.NCylindrical
        cdof = cdof + 4;
    end
    %Accounts for the translation
    for i = 1:Joints.NTranslation
        cdof = cdof + 5;
    end
    %Account for the simple constrains
    for i = 1:Joints.NSimple
        cdof = cdof + 1;
    end
    
    cdof = cdof + NBodies - 1; %Euler Parameter Constraints
    cdof = cdof + Joints.NDriver; %Each Driver accounts for 1 Dof
    
    numberofdofs = NBodies*bdof;
    
    OverallDofs = numberofdofs - cdof;
    
    %Store debug dataData
    debugdata(1).tdof = numberofdofs; 
    debugdata(1).cdof = cdof;
    debugdata(1).odof = OverallDofs;
    
%     if OverallDofs ~= 0
%         disp('The System is not porperly constrain, i.e, is not equal to zero, do you wish to proceed?')
%         reply = input('Continue? y/n','s');
%         if reply ~= 'y'
%             return
%         end
%     end

end

