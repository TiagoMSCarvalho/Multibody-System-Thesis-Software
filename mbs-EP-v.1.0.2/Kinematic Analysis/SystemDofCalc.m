function [debugdata] = SystemDofCalc(NBodies,Joints,debugdata,SimType,coord)
%This function calculates the number of DoFs to ensure that the multibody is properly constrained;
    %This function is also responsible for warning the user for an issue
    %with the contrains to facilitate the problem debug.
    %Keep in mind that for Kinematic Analyis the Jacobian matrix must be
    %squared and the odofs result should be 0, (Wichita Thesis)
    if strcmp(SimType,"Kin") || strcmp(SimType,"kin") || coord == 7
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
        %Accounts for the Sph-Sph
        for i = 1:Joints.NCompSpherical
            cdof = cdof + 2;
        end
        %Accounts for the Sph-Rev
        for i = 1:Joints.NSphRev
            cdof = cdof + 1;
        end
        %Accounts for the Tra-Rev
        for  i = 1:Joints.NTraRev
            cdof = cdof + 4;
        end

        cdof = cdof + NBodies - 1; %Euler Parameter Constraints
        cdof = cdof + Joints.NDriver; %Each Driver accounts for 1 Dof

        numberofdofs = NBodies*bdof;

        OverallDofs = numberofdofs - cdof;

        %Store debug dataData
        debugdata.tdof = numberofdofs; 
        debugdata.cdof = cdof;
        debugdata.odof = OverallDofs;
    elseif strcmp(SimType,"Dyn") || strcmp(SimType,"dyn") || coord ==6
        bdof = 6;
        cdof = 6;
        
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
        %Accounts for the Sph-Sph
        for i = 1:Joints.NCompSpherical
            cdof = cdof + 1;
        end
        %Accounts for the Sph-Rev
        for i = 1:Joints.NSphRev
            cdof = cdof + 2;
        end
        %Accounts for the Tra-Rev
        for  i = 1:Joints.NTraRev
            cdof = cdof + 4;
        end
        
        cdof = cdof + Joints.NDriver; %Each Driver accounts for 1 Dof

        numberofdofs = NBodies*bdof;

        OverallDofs = numberofdofs - cdof;

        %Store debug dataData
        debugdata.tdof = numberofdofs; 
        debugdata.cdof = cdof;
        debugdata.odof = OverallDofs;
    end

end

