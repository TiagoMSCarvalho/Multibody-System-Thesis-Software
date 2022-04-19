 function [vectorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time)
%Assemblies the Vector g to enable the calculation of the initial
%acceleration
%Pre Allocation of the Vectors
vectorg = zeros(6*NBodies,1);
forceel = zeros(6*NBodies,1);
%Allocation of the gravity properties and vector
gdir = string(Grav.Direction);
gmag = cell2mat(Grav.Magnitude);
if strcmp(gdir,"x") == 1
    g = [gmag;0;0]; %kgmms^-2
elseif strcmp(gdir,"y") == 1
    g = [0;gmag;0];
elseif strcmp(gdir,"z") == 1
    g = [0;0;gmag];
end

%Calculus of the Force Elements
for forcescount = 1:Forces.NSpring
    [forceel] = Force_Spring(forcescount,Bodies,Forces.Spring);
end
for forcescount = 1:Forces.NTSpring
    [forceel] = Force_TSpring(forcescount,Bodies,Forces.TSpring);
end
for forcescount = 1:Forces.NDamper
    [forceel] = Force_Damper(forcescount,Bodies,Forces.Damper);
end
%Allocation of the forces to the bodies and calculus of: swJw and weight
for i = 1:NBodies
    %% Moment of Inertia Calculus and Inertia covertion to global (taking into acc body mov)
    Mass = Bodies(i).Mass;
    Inertia = Bodies(i).Inertia;
    A = Bodies(i).A;
    I = diag(Inertia);
    w = Bodies(i).w;
    sw = SkewMatrix3(w);
    Ia = A*I*A';
    wJw = sw*Ia*w;
    %% Calculus of the Force and Torque Vector - isnumeric/ischar ensures that the user can use a function has input or scalars.
    % Force
    forcevec = zeros(1,3);
    cellforce = dynfunc(i).forcefunc;
    for j1 = 1:3
        if isnumeric(cellforce{1,j1})
            if strcmp(UnitsSystem,"mmks") == 1  || strcmp(UnitsSystem,"MMKS") == 1
                forcevec(1,j1) = cellforce{1,j1}*10^3;
            elseif strcmp(UnitsSystem,"si") == 1 || strcmp(UnitsSystem,"SI") == 1 || strcmp(UnitsSystem,"MKS") == 1 || strcmp(UnitsSystem,"mks") == 1
                forcevec(1,j1) = cellforce{1,j1};
            end
        elseif ischar(cellforce{1,j1})
            inputfunc = convertCharsToStrings(cellforce{1,j1});
            inputfunc = str2func(inputfunc);            
            if strcmp(UnitsSystem,"mmks") == 1  || strcmp(UnitsSystem,"MMKS") == 1
                forcevec(1,j1) = inputfunc(time)*10^3;
            elseif strcmp(UnitsSystem,"si") == 1 || strcmp(UnitsSystem,"SI") == 1 || strcmp(UnitsSystem,"MKS") == 1 || strcmp(UnitsSystem,"mks") == 1
                forcevec(1,j1) = inputfunc(time);
            end
        end
    end
    % Torque
    forcetor = zeros(1,3);
    celltorque = dynfunc(i).torquefunc;
    for j2 = 1:3
        if isnumeric(celltorque{1,j2})
            if strcmp(UnitsSystem,"mmks") == 1  || strcmp(UnitsSystem,"MMKS") == 1
                forcetor(1,j2) = celltorque{1,j2}*10^3;
            elseif strcmp(UnitsSystem,"si") == 1 || strcmp(UnitsSystem,"SI") == 1 || strcmp(UnitsSystem,"MKS") == 1 || strcmp(UnitsSystem,"mks") == 1
                forcetor(1,j2) = celltorque{1,j2};
            end
        elseif ischar(celltorque{1,j2})
            inputfunc = convertCharsToStrings(celltorque{1,j2});
            inputfunc = str2func(inputfunc);
            if strcmp(UnitsSystem,"mmks") == 1  || strcmp(UnitsSystem,"MMKS") == 1
                forcetor(1,j2) = inputfunc(time)*10^3;
            elseif strcmp(UnitsSystem,"si") == 1 || strcmp(UnitsSystem,"SI") == 1 || strcmp(UnitsSystem,"MKS") == 1 || strcmp(UnitsSystem,"mks") == 1
                forcetor(1,j2) = inputfunc(time);
            end
        end
    end
    % Calculus of the moment created by Forces not applied to the CoM
    if all(Bodies(i).ForcePoA == 0)
        ForceMoment = zeros(1,3);
    else
        PoA = Bodies(i).ForcePoA - Bodies(i).r;
        ForceMoment = cross(PoA,forcevec);
    end
    %% Allocation of the force vectors values
    i1 = 6*(i-1)+1;
    if isnan(gmag)
        vectorg(i1:i1+2,1) = Impose_Column(forcevec);
        vectorg(i1+3:i1+5,1) = Impose_Column(forcetor) + Impose_Column(ForceMoment) - Impose_Column(wJw);        
    elseif ~isnan(gmag)
        vectorg(i1:i1+2,1) = Impose_Column(forcevec) + Mass*Impose_Column(g);
        vectorg(i1+3:i1+5,1) = Impose_Column(forcetor) + Impose_Column(ForceMoment) - Impose_Column(wJw);
    end
end

vectorg = vectorg + forceel;

end

