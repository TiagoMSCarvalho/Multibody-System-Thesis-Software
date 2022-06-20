 function [vectorg] = ForceCalculus(Forces,NBodies,Bodies,dynfunc,Grav,UnitsSystem,time,ForceFunction)
%Assemblies the Vector g to enable the calculation of the initial
%acceleration
%Pre Allocation of the Vectors (Euler Parameters)
vectorg = zeros(7*NBodies,1);
forceel1 = zeros(7*NBodies,1);
forceel2 = zeros(7*NBodies,1);
forceel3 = zeros(7*NBodies,1);
forceel4 = zeros(7*NBodies,1);

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
    [forceel1] = Force_Spring(forcescount,NBodies,Bodies,Forces.Spring,ForceFunction,forceel1);
end
for forcescount = 1:Forces.NTSpring
    [forceel2] = Force_TSpring(forcescount,NBodies,Bodies,Forces.TSpring,forceel2);
end
for forcescount = 1:Forces.NDamper
    [forceel3] = Force_Damper(forcescount,NBodies,Bodies,Forces.Damper,ForceFunction,forceel3);
end
for forcescount = 1:Forces.NActuator
    [forceel4] = Force_Actuator(forcescount,NBodies,Bodies,Forces.Actuator,ForceFunction,time,forceel4);
end
%Allocation of the forces to the bodies and calculus of: swJw and weight
for i = 1:NBodies
    %% Moment of Inertia Calculus and Inertia covertion to global (taking into acc body mov)
    Mass = Bodies(i).Mass;
    Mass = eye(3)*Mass;
    Inertia = Bodies(i).Inertia;
    I = diag(Inertia);
    w = Bodies(i).w;
    pd = 0.5*Bodies(i).L'*w;
    pd = Impose_Column(pd);
    %Definition of Ld
    e0d = pd(1); 
    e1d = pd(2);
    e2d = pd(3);
    e3d = pd(4);
    e = [e1d;e2d;e3d];
    Ld = [-e,-SkewMatrix3(e) + e0d*eye(3)];
    %Calculo do wJw para EP
    wJw = 8*Ld'*I*Bodies(i).L*pd;
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
            sym t;
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
            sym t;
            if strcmp(UnitsSystem,"mmks") == 1  || strcmp(UnitsSystem,"MMKS") == 1
                forcetor(1,j2) = inputfunc(time)*10^3;
            elseif strcmp(UnitsSystem,"si") == 1 || strcmp(UnitsSystem,"SI") == 1 || strcmp(UnitsSystem,"MKS") == 1 || strcmp(UnitsSystem,"mks") == 1
                forcetor(1,j2) = inputfunc(time);
            end
        end
    end
    forcetor = 2*Bodies(i).L'*forcetor;
    % Calculus of the moment created by Forces not applied to the CoM
    if all(Bodies(i).ForcePoA == 0) || isnan(Bodies(i).ForcePoA)
        ForceMoment = zeros(1,4);
    else
        PoA = Bodies(i).ForcePoA - Bodies(i).r;
        ForceMoment = 2*Bodies(i).L'*(cross(PoA,forcevec));
    end
    %% Allocation of the force vectors values
    i1 = 7*(i-1)+1;
    if isnan(gmag)
        vectorg(i1:i1+2,1) = Impose_Column(forcevec);
        vectorg(i1+3:i1+6,1) = Impose_Column(forcetor) + Impose_Column(ForceMoment) - Impose_Column(wJw);        
    elseif ~isnan(gmag)
        vectorg(i1:i1+2,1) = Impose_Column(forcevec) + Mass*Impose_Column(g);
        vectorg(i1+3:i1+6,1) = Impose_Column(forcetor) + Impose_Column(ForceMoment) - Impose_Column(wJw);
    end
end

vectorg = vectorg + forceel1 + forceel2 + forceel3 + forceel4;

end

