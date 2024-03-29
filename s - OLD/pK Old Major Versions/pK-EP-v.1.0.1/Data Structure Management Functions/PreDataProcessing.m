% This function read the excel file with a structure which can be seen in
% the actual document. The purpose is to define a array of structs for the
% Joint and the Bodies.  

% For the Joints, for now, the information is Joint
% Type, Body1 (number of body), Body2, and the three coordinates of two different points
% (second one is optional). This points mean different things for different
% joints.

% For the Bodies, for now, the information is Body Name and the Joints in
% which the Bodies are involved (number of joint)
function [Bodies, Joints, Motions,debugdata,ang] = PreDataProcessing(filename,JointTypes)
[Bodies,~,debugdata,ang] = ReadBodiesInfo(filename);
Joints = ReadJointsInfo(filename,Bodies);
Motions = ReadMotionsInfo(filename);%reads the number of time iterations and motions
end

function [Bodies, NBodies,debugdata,ang] = ReadBodiesInfo(filename)
    %Prompts the user to state the type of inputs (Angles or Axis Vectors)
    %Temporary before GUI
    prompt = 'State the type of input for the body frame orientation [Axis Vectors/Bryant Angles/Orientational Axis]:';
    str = input(prompt,'s');
    prompt2 = 'Angles are in Rad or Deg ? [Rad/Deg]';
    ang = input(prompt2,'s');

    % Read all the information relative to the bodies in the kinematics program
    [~,BodyNames,~] = xlsread(filename,'Bodies','B4:B100');
    BodyInfo = xlsread(filename,'Bodies','C4:V100');
    %Separates the Information by Column transforms into row vectors
    Origin = BodyInfo(:,1:3);
    if strcmp(str,'Axis Vectors') == 1
        Point_along_x = BodyInfo(:,4:6);
        Point_along_y = BodyInfo(:,7:9);
    elseif strcmp(str,'Bryant Angles') == 1
        eang = BodyInfo(:,10:12);
    elseif strcmp(str,'Orientational Axis') == 1
        uaxis = BodyInfo(:,13:15);
        rota = BodyInfo(:,16);
    else
        str = input(prompt,'s');
    end
    Mass = BodyInfo(:,17);
    Inertia = BodyInfo(:,18:20);
    % Number of bodies for cycle
    NBodies = size(Origin,1); %Returns Number of Rows ( 1st Dim)
    debugdata = struct([]); %Preallocates the debugdata struct
    
    for i=1:NBodies
        % Orientational Axis doesn't need to calculate the A matrix.
        Bodies(i).Name = BodyNames{i};
        Bodies(i).r = Impose_Column(Origin(i,:)); 
        if strcmp(str,'Axis Vectors') == 1
            A = AxisVectorsAmatrix(i,Point_along_x,Point_along_y,Origin);
            [Bodies(i).p,debugdata] = EulerParameters(A);
        elseif strcmp(str,'Bryant Angles') == 1
            A = BryantAnglesAmatrix(i,eang,ang);
            [Bodies(i).p,debugdata] = EulerParameters(A);
        elseif strcmp(str,'Orientational Axis') == 1
            [Bodies(i).p,debugdata] = OrientationalAxisAmatrix(i,uaxis,rota,ang);
        end
        Bodies(i).Mass = Mass(i);
        Bodies(i).Inertia = Impose_Column(Inertia(i,:));
    end
end

function A = AxisVectorsAmatrix(i,Point_along_x,Point_along_y,Origin)
%Produces the A matrix for the Axis Vectors Input
%Axis Vectors is now robust for points on x and points in y that do not
%form perpendicular vectors
        
        % 2 points from solidworks that define the body plan
        vector1 = Impose_Column(Point_along_x(i,:) - Origin(i,:));
        vector2 = Impose_Column(Point_along_y(i,:) - Origin(i,:));
        
        % Determination of the normal vector do the body plan
        v1norm = vector1/norm(vector1);
        v2norm = vector2/norm(vector2); 
        z_norm = cross(v1norm,v2norm);
        
        %Plan equation
        planeq = z_norm(1)*(x-Origin(i,1)) + z_norm(2)*(y-Origin(i,2)) + z_norm(3)*(z-Origin(i,3));
        x_norm = v1norm; %Attribuition of the x axis.
        
        %Determination of the y axis (perpendicular to z)
        y_vector = coss(x_norm,z_norm);
        y_norm = y_vector/norm(y_vector);
        
        x = y_norm(1);
        y = y_norm(2);
        z = y_norm(3);
        
        if planeq == 0 
            A = [x_norm,y_norm,z_norm];
        else
            disp('error')
        end
end

function A = BryantAnglesAmatrix(i,eang,ang)
%Produces the A matrix for the Bryant Angles
    eangv = Impose_Column(eang(i,:));

    if strcmp(ang,'Rad') ~= 1 && strcmp(ang,'Deg') ~= 1
        ang = input(prompt2,'s');
    elseif strcmp(ang,'Deg') == 1
        roll = deg2rad(eangv(1));
        pitch = deg2rad(eangv(2));
        yaw = deg2rad(eangv(3));
    elseif strcmp(ang,'Rad') == 1
        roll = eangv(1);
        pitch = eangv(2);
        yaw = eangv(3);
    end

    % Nikravesh page 351
    D = [1,0,0;
        0,cos(roll),-sin(roll);
        0,sin(roll),cos(roll)];

    C = [cos(pitch),0,sin(pitch);
        0,1,0;
        -sin(pitch),0,cos(pitch)];

    B = [cos(yaw),-sin(yaw),0;
        sin(yaw),cos(yaw),0;
        0,0,1];

    A = D*C*B;
end

function [epar,debugdata] = OrientationalAxisAmatrix(i,uaxis,rota,ang)
%Produces the A matrix and the vector of euler parameters for the Orientational Axis input
    uaxisc = Impose_Column(uaxis(i,:));

    if strcmp(ang,'Rad') ~= 1 && strcmp(ang,'Deg') ~= 1
        ang = input(prompt2,'s');
    elseif strcmp(ang,'Deg') == 1
        rotan = deg2rad(rota(i));
    elseif strcmp(ang,'Rad') == 1
        rotan = rota(i);
    end
    
    e0 = cos(rotan/2);
    e = uaxisc' * sin(rotan/2);
    e1 = e(1);
    e2 = e(2);
    e3 = e(3);

    A = 2.*[e0^2+e1^2-(1/2), e1*e2-e0*e3, e1*e3+e0*e2;
        e1*e2+e0*e3, e0^2+e2^2-(1/2), e2*e3-e0*e1;
        e1*e3-e0*e2, e2*e3+e0*e1, e0^2+e3^2-(1/2)];
    
    debugdata(1).AtA = A'*A;
    
    epar = [e0;e1;e2;e3];
    j = epar'*epar;
    j = round(j,0);

    if j ==1
        disp('Euler Parameter Condition Reached');
    else
        disp('Euler Parameter Condition not Reached');
        return
    end
end

% Euler Parameters
function [epar,debugdata] = EulerParameters(A)
%Uses the A matrix previosly defined from T1 and T2 inputs and calculates
%the euler parameter vector.

    %stores data that allows easy debug of the program
    debugdata(1).AtA = A'*A;

    %EULER PARAMETERS - All equations retrieved from page 162 from Nikravesh
    %If the sign of e0 is inverted, the signs e1,e2,e3 will be inverted also.
    %Changing the signs of all 4 parameters doesn't influence the A matrix
    %since it is quadratic.
    tr = trace(A);
    a = (tr+1)/4;
    e0 = sqrt(a);
    
    if e0 == 0
        b = (1 + 2*A(1,1) - tr)/4;
        e1 = sqrt(b); 
        e2 = (A(2,1) + A(1,2))/(4*e1);
        e3 = (A(3,1) + A(1,3))/(4*e1);
    elseif e0 ~= 0
        e1 = (A(3,2) - A(2,3))/(4*e0);
        e2 = (A(1,3) - A(3,1))/(4*e0);
        e3 = (A(2,1) - A(1,2))/(4*e0);
    end

    epar = [e0;e1;e2;e3];
    j = epar'*epar;
    j = round(j,0);

    if j ==1
        disp('Euler Parameter Condition Reached');
    else
        disp('Euler Parameter Condition not Reached');
        return
    end
end

function Joints = ReadJointsInfo(filename,Bodies)
% Read the excel sheet in the range with information
[~,~,raw] = xlsread(filename,'Joints','A2:N1000');
% Identify which lines of the spreadsheet contain joint information,
% removing header lines, spacing lines and lines after the final piece of
% information, Only consideres numerical
relevant_lines = [];
for i=1:size(raw,1) %Number of Lines of raw
    if isnumeric(raw{i,4})
        if ~isnan(raw{i,4}) %Not Nan -> 1 if it is not empty, it is 0 for strings
            relevant_lines = [relevant_lines,i]; %Stores that data line in this vector
        end
    end
end
% Vector of strings with the type of joint
JointTypes = raw(relevant_lines,2); %Column 2 has the type of joints, in strings (Excel)
% Matrix of doubles with all the joint numerical information to be parsed later
%cell2mat -> Converts the data to a matrix
JointInfo = cell2mat(raw(relevant_lines,4:14));
% Determine the number of Joints ans Bodies for the cycles
n_Joints = size(JointTypes,1); %Size (Struct,1) , 1st dim of the struct
% Initialize the joint type counts as 0
Joints.NSpherical = 0;
Joints.NUniversal = 0;
Joints.NRevolute = 0;
Joints.NCylindrical = 0;
Joints.NTranslation = 0;
Joints.NSimple = 0;
Joints.NGround = 0;
Joints.NDriver = 0;
Joints.NPoint = 0;

for i=1:n_Joints
    Joints = ProcessJoint(JointTypes{i},Joints,JointInfo(i,:),Bodies);
end
end

function [Motions] = ReadMotionsInfo(filename)%for the coordinate transf.

[~,~,Heave] = xlsread(filename,'Motions','D5');
[~,~,Roll] = xlsread(filename,'Motions','D6');
[~,~,Pitch] = xlsread(filename,'Motions','D7');
[~,~,RunTime] = xlsread(filename,'Motions','D10');
[~,~,TimeStep] = xlsread(filename,'Motions','D11');
[~,~,PitchDisplacement] = xlsread(filename,'Motions','N7');

Motions.Heave=Heave;
Motions.Roll=Roll;
Motions.Pitch=Pitch;
Motions.RunTime=RunTime;
Motions.TimeStep=TimeStep;
Motions.PitchDisplacement=PitchDisplacement;

end

function Joints = ProcessJoint(JointType,Joints,JointInfo,Bodies)
if strcmp(JointType,'Spherical')%strcmp, compares the retrieved string to the desired one
    Joints.NSpherical = Joints.NSpherical + 1; %increases the size of the vector
    Joints = ProcessSpherical(Joints,JointInfo,Joints.NSpherical, Bodies);
elseif strcmp(JointType,'Universal')
    Joints.NUniversal = Joints.NUniversal + 1;
    Joints = ProcessUniversal(Joints,JointInfo,Joints.NUniversal, Bodies);
elseif strcmp(JointType,'Revolute')
    Joints.NRevolute = Joints.NRevolute + 1;
    Joints = ProcessRevolute(Joints,JointInfo,Joints.NRevolute, Bodies);
elseif strcmp(JointType,'Cylindrical')
    Joints.NCylindrical = Joints.NCylindrical + 1;
    Joints = ProcessCylindrical(Joints,JointInfo,Joints.NCylindrical, Bodies);
elseif strcmp(JointType,'Translation')
    Joints.NTranslation = Joints.NTranslation + 1;
    Joints = ProcessTranslation(Joints,JointInfo,Joints.NTranslation, Bodies);
elseif strcmp(JointType,'Simple')
    Joints.NSimple = Joints.NSimple + 1;
    Joints = ProcessSimple(Joints,JointInfo,Joints.NSimple);
elseif strcmp(JointType,'Ground')
    Joints.NGround = Joints.NGround + 1;
    Joints = ProcessGround(Joints,JointInfo,Joints.NGround, Bodies);
elseif strcmp(JointType,'Driver')
    Joints.NDriver = Joints.NDriver + 1;
    Joints = ProcessDriver(Joints,JointInfo,Joints.NDriver);
elseif strcmp(JointType,'Point')
    Joints.NPoint = Joints.NPoint + 1;
    Joints = ProcessPoint(Joints,JointInfo,Joints.NPoint,Bodies);
end
end

function Joints = ProcessSpherical(Joints,JointsInfo,jointCount,Bodies)
% Body numbers
Joints.Spherical(jointCount).Body1 = JointsInfo(1); %Acces to Joints Struct and Spherical Sub Struct field Body and extract to a variable
Joints.Spherical(jointCount).Body2 = JointsInfo(2);
% Pass body numbers to easier to use variables
i = Joints.Spherical(jointCount).Body1;
j = Joints.Spherical(jointCount).Body2;
% Location of joint center in fixed reference
sp = Impose_Column(JointsInfo(3:5));
% Get euler parameter for each body frame
pi = Bodies(i).p;
pj = Bodies(j).p;
% Transform joint location on fixed reference to the bodies' local
% reference
spi = sp - Bodies(i).r;
spi = EarthtoBody(spi,pi);
spj = sp - Bodies(j).r;
spj = EarthtoBody(spj,pj);
% Save the joint location in each bodies' reference
Joints.Spherical(jointCount).spi = spi;
Joints.Spherical(jointCount).spj = spj;
end

function Joints = ProcessUniversal (Joints,JointsInfo,jointCount,Bodies)
% Body numbers
Joints.Universal(jointCount).Body1 = JointsInfo(1);
Joints.Universal(jointCount).Body2 = JointsInfo(2);
% Pass body numbers to easier to use variables
i = Joints.Universal(jointCount).Body1;
j = Joints.Universal(jointCount).Body2;
% Location of joint center and si and sj vectors in fixed reference
sp = Impose_Column(JointsInfo(3:5));
si_earth = Impose_Column(JointsInfo(6:8));
sj_earth = Impose_Column(JointsInfo(9:11));
% Get euler parameter for each body frame
pi = Bodies(i).p;
pj = Bodies(j).p;
% Transform joint location, si and sj on fixed reference to the bodies' local
% reference
spi = sp - Bodies(i).r; % r -> Vector to the origin of the body frame
spi = EarthtoBody(spi,pi);
spj = sp - Bodies(j).r;
spj = EarthtoBody(spj,pj);
si_earth = si_earth - sp;
sj_earth = sj_earth - sp;
si = EarthtoBody(si_earth,pi);
sj = EarthtoBody(sj_earth,pj);
% Save the joint location in each bodies' reference
Joints.Universal(jointCount).spi = spi;
Joints.Universal(jointCount).spj = spj;
Joints.Universal(jointCount).si = si;
Joints.Universal(jointCount).sj = sj;
end

function Joints = ProcessRevolute (Joints,JointsInfo,jointCount,Bodies)
% Body numbers
Joints.Revolute(jointCount).Body1 = JointsInfo(1);
Joints.Revolute(jointCount).Body2 = JointsInfo(2);
% Pass body numbers to easier to use variables
i = Joints.Revolute(jointCount).Body1;
j = Joints.Revolute(jointCount).Body2;
% Location of joint center and axis vector in fixed reference
sp = Impose_Column(JointsInfo(3:5));
s = Impose_Column(JointsInfo(6:8));
% Get euler parameter for each body frame
pi = Bodies(i).p;
pj = Bodies(j).p;
% Transform joint location and axis vector on fixed reference to the bodies' local
% reference
spi = sp - Bodies(i).r;
spi = EarthtoBody(spi,pi);
spj = sp - Bodies(j).r;
spj = EarthtoBody(spj,pj);
si = EarthtoBody(s,pi);
sj = EarthtoBody(s,pj);
% Save the joint location in each bodies' reference
Joints.Revolute(jointCount).spi = spi;
Joints.Revolute(jointCount).spj = spj;
Joints.Revolute(jointCount).si = si;
Joints.Revolute(jointCount).sj = sj;
end

function Joints = ProcessCylindrical (Joints,JointsInfo,jointCount,Bodies)
% Body numbers
Joints.Cylindrical(jointCount).Body1 = JointsInfo(1);
Joints.Cylindrical(jointCount).Body2 = JointsInfo(2);
% Pass body numbers to easier to use variables
i = Joints.Cylindrical(jointCount).Body1;
j = Joints.Cylindrical(jointCount).Body2;
% Location of joint center and axis vector in fixed reference
sp = Impose_Column(JointsInfo(3:5));
s = Impose_Column(JointsInfo(6:8));
% Get euler parameter for each body frame
pi = Bodies(i).p;
pj = Bodies(j).p;
% Transform joint location and axis vector on fixed reference to the bodies' local
% reference
spi = sp - Bodies(i).r;
spi = EarthtoBody(spi,pi);
spj = sp - Bodies(j).r;
spj = EarthtoBody(spj,pj);
si = EarthtoBody(s,pi);
sj = EarthtoBody(s,pj);
% Save the joint location in each bodies' reference
Joints.Cylindrical(jointCount).spi = spi;
Joints.Cylindrical(jointCount).spj = spj;
Joints.Cylindrical(jointCount).si = si;
Joints.Cylindrical(jointCount).sj = sj;
end

function Joints = ProcessTranslation (Joints,JointsInfo,jointCount,Bodies)
% Body numbers
Joints.Translation(jointCount).Body1 = JointsInfo(1);
Joints.Translation(jointCount).Body2 = JointsInfo(2);
% Pass body numbers to easier to use variables
i = Joints.Translation(jointCount).Body1;
j = Joints.Translation(jointCount).Body2;
% Location of joint center and axis vector in fixed reference
sp = Impose_Column(JointsInfo(3:5));
s = Impose_Column(JointsInfo(6:8));
% hi_earth = [s(2);-s(1);0];
% hj_earth = cross(s,hi_earth);
% Get euler parameters for each bodies' reference
pi = Bodies(i).p;
pj = Bodies(j).p;
% Transform joint location and axis vector on fixed reference to the bodies' local
% reference
spi = sp - Bodies(i).r;
spi = EarthtoBody(spi,pi);
spj = sp - Bodies(j).r;
spj = EarthtoBody(spj,pj);
si = EarthtoBody(s,pi);
sj = EarthtoBody(s,pj);
% Save the joint location in each bodies' reference
Joints.Translation(jointCount).spi = spi;
Joints.Translation(jointCount).spj = spj;
Joints.Translation(jointCount).si = si;
Joints.Translation(jointCount).sj = sj;
end

function Joints = ProcessSimple (Joints,JointsInfo,jointCount)
% Body number
Joints.Simple(jointCount).Body = JointsInfo(1);
% Body position and degree constrained
Joints.Simple(jointCount).pos0 = JointsInfo(2);
Joints.Simple(jointCount).direction = JointsInfo(3);
end

function Joints = ProcessGround (Joints,JointsInfo,jointCount,Bodies)
% Body number
Joints.Ground(jointCount).Body = JointsInfo(1);
i = JointsInfo(1);
% Body position and degree constrained
Joints.Ground(jointCount).r0 = Bodies(i).r;
Joints.Ground(jointCount).p0 = Bodies(i).p;
end

function Joints = ProcessDriver (Joints,JointsInfo,jointCount)
% Body number
Joints.Driver(jointCount).Body = JointsInfo(1);
% Body position and degree constrained
Joints.Driver(jointCount).pos0 = JointsInfo(2);
Joints.Driver(jointCount).v0 = JointsInfo(3);
Joints.Driver(jointCount).a0 = JointsInfo(4);
Joints.Driver(jointCount).direction = JointsInfo(5);
if JointsInfo(5) > 3
    Joints.Driver(jointCount).rotaxis = JointsInfo(6:8);
end
end

function Joints = ProcessPoint(Joints,JointsInfo,jointCount,Bodies) 
% Body number
Joints.Point(jointCount).Body = JointsInfo(1);
i = JointsInfo(1);
% Point Location
sp = Impose_Column(JointsInfo(2:4));
pi = Bodies(i).p;
spi = sp - Bodies(i).r;
spi = EarthtoBody(spi,pi);
Joints.Point(jointCount).spi = spi;
end