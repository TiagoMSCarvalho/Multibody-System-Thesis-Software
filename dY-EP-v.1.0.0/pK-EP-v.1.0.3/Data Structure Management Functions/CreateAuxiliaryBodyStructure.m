%% Create Auxiliary Body Structure Function Details
%
% Inputs and Outputs:
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - q: vector with all the positions and angular orientations of the bodies
%    in the same order as in the Bodies structure. This vector is used to
%    solve the position analysis with fsolve.
%
% Objective of the Function:
%    This function creates the vector q of positions and orientations of
%    the vectors from the Bodies structure to use on the solving of te
%    positions analysis


function q = CreateAuxiliaryBodyStructure(NBodies,Bodies)

% para i = 1 -> il = 6*0+1 -> q(1:3) = Bodies(1).r ( field w vector w/ 3
% components) same for the euler parameters but q(4:7) = bodies(1).p ( field w/ vector
% / 4 components)
% for i= 2 -> il= 7*1 +1 ( 7 -> que são so campos ocupados anteriormente)

    for i=1:NBodies
        l1 = 7*(i-1)+1;
        q(l1:l1+2,1) = Bodies(i).r;
        q(l1+3:l1+6,1) = Bodies(i).p;
    end
%Corrigido para Parametros de Euler
end
