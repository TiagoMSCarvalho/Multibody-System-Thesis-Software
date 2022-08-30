function [CoM,Points,it] = KinDataStorage(CoM,Points,NBodies,Bodies,Joints,vel,acc,it)
%Store data from each iteration regarding the points to be study.

    %% Creates a struct name Data to store Point and CoM Position, Vel, Acc
    DataP = struct([]);
    DataC = struct([]);
    %% Storage of the Center of Massa Data
    for k = 1:NBodies
        DataC(k).Position = Bodies(k).r;
        if it > 1
            DataC(k).TraVel = Bodies(k).rd;
            DataC(k).AngVel = Bodies(k).w;
            DataC(k).TraAccel = Bodies(k).rdd;
            DataC(k).AngAccel = Bodies(k).wd;
        end
    end
    %% Storage of the Joints Data
    %% Calculus of the Position of each Joint
    for k = 1:Joints.NPoint %Acces the NPoint field of the Joints Struct
        i = Joints.Point(k).Body;
        spi = Joints.Point(k).spi; % spi - point position vector
        sp_earth = BodytoEarth(spi,Bodies(i).p);
        Point_position = Bodies(i).r + sp_earth; %pela expressao do d, r até a origem do referencial sp no ref global ate p
        DataP(k).Position = Point_position;
    end
    %% Calculus of the Joints Velocity and Acceleration Using Basic 3D Rigid Body Kinematics (Separated from the above for easier debug)
        %Used Equations:
            % Velocity - Va = Vb + w x R(A/B)
            % Acceleration -  Aa = Ab + alpha x R(A/B) + w x (w xR(A/B))
    if it > 1
        for k = 1:Joints.NPoint
            %% Velocity Calculus
            i = Joints.Point(k).Body;
            spi = Joints.Point(k).spi;
            spi_earth = BodytoEarth(spi,Bodies(i).p); %R(A/b)
            i1 = 6*(i-1)+1;
            rd = Impose_Column(vel(i1:i1+2));
            w = Impose_Column(vel(i1+3:i1+5));
            rdp = rd + cross(w,spi_earth);
            DataP(k).TraVel = Impose_Column(rdp);
            DataP(k).AngVel = w;
            %% Acceleration Calculus
            rdd = Impose_Column(acc(i1:i1+2));
            wd = Impose_Column(acc(i1+3:i1+5));
            cwr = cross(w,spi_earth);
            rddp = rdd + cross(wd,spi_earth) + cross(w,cwr);
            DataP(k).TraAccel = Impose_Column(rddp);
            DataP(k).AngAccel = wd;
        end
    end
    %% Stores said data struct in a cell array per number of iteration
    CoM{1,it} = DataC;
    Points{1,it} = DataP;
    it = it+1;
end