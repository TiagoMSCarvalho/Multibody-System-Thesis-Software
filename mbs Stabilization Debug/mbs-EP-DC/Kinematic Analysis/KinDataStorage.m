function [Points,it] = KinDataStorage(Points,Bodies,Joints,vel,acc,it)
%Store data from each iteration regarding the points to be study.

    %% Creates a struct name Data to store Point Position, Vel, Acc
    Data = struct([]);
    %% Calculus of the Position of each Joint
    for k = 1:Joints.NPoint %Acces the NPoint field of the Joints Struct
        i = Joints.Point(k).Body;
        spi = Joints.Point(k).spi; % spi - point position vector
        sp_earth = BodytoEarth(spi,Bodies(i).p);
        Point_position = Bodies(i).r + sp_earth; %pela expressao do d, r até a origem do referencial sp no ref global ate p
        Data(k).Position = Point_position;
    end
    %% Calculus of the Joints Velocity and Acceleration Using Basic 3D Rigid Body Kinematics (Separated from the above for easier debug)
        %Used Equations:
            % Velocity - Va = Vb + w x R(A/B)
            % Acceleration -  Aa = Ab + alpha x R(A/B) + w x (w xR(A/B))
    poivel = []; %Points of Interest Velocity
    poiacc = []; %Points of Interest Acceleration
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
    end
    %% Store Vel and Accel Vectors in the fields
    [a,~] = size(vel);
    for j = 1:a
        Data(j).Velocity = poivel(j);
        Data(j).Acceleration = poiacc(j);
    end
    %% Stores said data struct in a cell array per number of iteration
    Points{1,it} = Data;
    it = it+1;
end