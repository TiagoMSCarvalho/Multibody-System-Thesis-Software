function [Points,CoM,it] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,acc,it)
%Stores data from each iteration regarding the points to be study.

    %% Creates a struct name Data to Store
    DataP = struct([]);
    DataC = struct([]);
    comvel = []; %Center of Mass Velocity
    comacc = []; %Center of Mass Acceleration
    poivel = []; %Points of Interest Velocity
    poiacc = []; %Points of Interest Acceleration
    %% Storage of the Center of Mass Data
    for k = 1:NBodies
        i1 = 6*(k-1)+1;
        DataC(k).Position = Bodies(k).r;
        comvel(i1:i1+2,1) = Bodies(k).rd;
        comvel(i1+3:i1+5,1) = Bodies(k).w;
        comacc(i1:i1+2,1) = Bodies(k).rdd;
        comacc(i1+3:i1+5,1) = Bodies(k).wd;
    end
    [a,~] = size(comvel);
    for j = 1:a
        DataC(j).Velocity = comvel(j);
        DataC(j).Acceleration = comacc(j);
    end
    %% Storage of the Joints Data
    % Storage of the velocity values in a vector
    % Used Equations:
            % Velocity - Va = Vb + w x R(A/B)
            % Acceleration -  Aa = Ab + alpha x R(A/B) + w x (w xR(A/B))
    for i = 1:NBodies
        i1 = 6*(i-1)+1;
        vel(i1:i1+2,1) = Bodies(i).rd;
        vel(i1+3:i1+5,1) = Bodies(i).w;
    end
    % Position
    for k = 1:Joints.NPoint
        %% Position Calculus
        i = Joints.Point(k).Body;
        spi = Joints.Point(k).spi; % spi - point position vector
        spi_earth = BodytoEarth(spi,Bodies(i).p);
        Point_position = Bodies(i).r + spi_earth;
        DataP(k).Position = Point_position;
        %% Velocity Calculus
        i1 = 6*(i-1)+1;
        rd = Impose_Column(vel(i1:i1+2));
        w = Impose_Column(vel(i1+3:i1+5));
        rdp = rd + cross(w,spi_earth);
        poivel(i1:i1+2,1) = Impose_Column(rdp);
        poivel(i1+3:i1+5,1) = w;
        %% Acceleration Calculus
        rdd = Impose_Column(acc(i1:i1+2));
        wd = Impose_Column(acc(i1+3:i1+5));
        cwr = cross(w,spi_earth);
        rddp = rdd + cross(wd,spi_earth) + cross(w,cwr);
        poiacc(i1:i1+2,1) = Impose_Column(rddp);
        poiacc(i1+3:i1+5,1) = wd;
    end
    [b,~] = size(acc);
    for j = 1:b
        DataP(j).Velocity = poivel(j);
        DataP(j).Acceleration = poiacc(j);
    end    
    %% Stores said data struct in a cell array per number of iteration
    Points{1,it} = DataP;
    CoM{1,it} = DataC;
    it = it+1;
end

