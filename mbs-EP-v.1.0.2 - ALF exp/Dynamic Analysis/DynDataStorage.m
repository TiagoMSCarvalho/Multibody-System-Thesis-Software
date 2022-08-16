function [Points,CoM,it] = DynDataStorage(Points,CoM,NBodies,Bodies,Joints,acc,it)
%Stores data from each iteration regarding the points to be study.

    %% Creates a struct name Data to Store
    DataP = struct([]);
    DataC = struct([]);
    %% Storage of the Center of Mass Data
    for k = 1:NBodies
        DataC(k).Position = Bodies(k).r;
        DataC(k).TraVel = Bodies(k).rd;
        DataC(k).AngVel = Bodies(k).w;
        DataC(k).TraAccel = Bodies(k).rdd;
        DataC(k).AngAccel = Bodies(k).wd;
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
        poivel = Impose_Column(rdp);
        % Velocity Storage
        DataP(k).TraVel = poivel;
        DataP(k).AngVel = w;
        %% Acceleration Calculus
        rdd = Impose_Column(acc(i1:i1+2));
        wd = Impose_Column(acc(i1+3:i1+5));
        cwr = cross(w,spi_earth);
        rddp = rdd + cross(wd,spi_earth) + cross(w,cwr);
        poiacc = Impose_Column(rddp);
        DataP(k).TraAccel = poiacc;
        DataP(k).AngAccel = wd;
        % Acceleration Storage
    end
%% Storage of the Previous Data correspondent for each iteration
    CoM{1,it} = DataC;
    Points{1,it} = DataP;
    it = it+1;
end

