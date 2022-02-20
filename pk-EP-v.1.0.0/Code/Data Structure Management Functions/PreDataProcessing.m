% This function read the excel file with a structure which can be seen in
% the actual document. The purpose is to define a array of structs for the
% Joint and the Bodies.  

% For the Joints, for now, the information is Joint
% Type, Body1 (number of body), Body2, and the three coordinates of two different points
% (second one is optional). This points mean different things for different
% joints.

% For the Bodies, for now, the information is Body Name and the Joints in
% which the Bodies are involved (number of joint)
function [Bodies, Joints, Motions] = PreDataProcessing(filename,JointTypes)
Bodies = ReadBodiesInfo(filename);
Joints = ReadJointsInfo(filename,Bodies);
Motions = ReadMotionsInfo(filename);%reads the number of time iterations and motions
end

function [Bodies, NBodies] = ReadBodiesInfo(filename)
    % Read all the information relative to the bodies in the kinematics program
    [~,BodyNames,~] = xlsread(filename,'Bodies','B3:B100');
    BodyInfo = xlsread(filename,'Bodies','C3:O100');
    %Separates the Information by Column transforms into row vectors
    Origin = BodyInfo(:,1:3);
    Point_along_x = BodyInfo(:,4:6);
    Point_along_y = BodyInfo(:,7:9);
    Mass = BodyInfo(:,10);
    Inertia = BodyInfo(:,11:13);
    % Number of bodies for cycle
    NBodies = size(Origin,1); %Returns Number of Rows ( 1st Dim)
    
    for i=1:NBodies
        % Point_along x, é um ponto no corpo
        % x_vector, y_vector vão dar vetores usados para calcular
        % orientacao do corpo
        Bodies(i).Name = BodyNames{i};
        Bodies(i).r = Impose_Column(Origin(i,:)); %Calls a External Function
        x_vector = Impose_Column(Point_along_x(i,:) - Origin(i,:));
        y_vector = Impose_Column(Point_along_y(i,:) - Origin(i,:));
        Bodies(i).p = EulerParameters(x_vector,y_vector);
        Bodies(i).Mass = Mass(i);
        Bodies(i).Inertia = Impose_Column(Inertia(i,:));
    end
end

% Euler Parameters
function epar = EulerParameters(x_vector,y_vector)
%Defines the matrix A for each body-frame: x and y vectors are defined through solidworks.

%Debug (18-01-2021):
%Previous way of doing the referentials Euler Parameters didn't pass the condition test
%further analysis of the excel sheet evidenced that x and y are not
%perpendicular. 

%Previous version
x_norm = x_vector/norm(x_vector); % norm() returns vector norm
y_norm = y_vector/norm(y_vector); 
z_norm = cross(x_norm,y_norm); % cross product gives a Perp vector to x and y norm plane

%New Version (18-01-2021) -> Use the x point to define the first axis and
%define 2 perpendicular vectores with the aux function.

% [~,x_norm]=unitvector(x_vector);
% [y_vector,z_vector] = PerpendicularVectors(x_norm);
% 
% [~,y_vector] = unitvector(y_vector);
% [~,z_vector] = unitvector(z_vector);

%     if norm(x_norm) == 1
%         disp('x is norm'),
%     elseif norm(x_norm) ~=1
%         disp('x is not norm')
%     end
%     
%     if norm(y_vector) == 1
%         disp('y is norm'),
%     elseif norm(y_vector) ~=1
%         disp('y is not norm')
%     end
%     
%     if norm(z_vector) == 1
%         disp('z is norm'),
%     elseif norm(z_vector) ~=1
%         disp('z is not norm')
%     end

%Code for debug (put in comment when done)
% h = dot(x_norm,y_norm);
% if h == 0
%     disp('Corr - x_norm and y_norm are perpendicular')
% elseif h~=0
%     disp('Wrong - x_norm and y_norm are not perpendicular')
% end

%Uses the body frame A matrix to retrieve the values of the Euler
%Parameters for each body
A = [x_norm,y_norm,z_norm];
A'*A;

%Nikravesh Det of Euler Parameters

%If the sign of e0 is inverted, the signs e1,e2,e3 will be inverted also.
%Changing the signs of all 4 parameters doesn't influence the A matrix
%since it is quadratic.

%All equations retrieved from page 162 from Nikravesh
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

% phi = 2* acos(e0);

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

function Joints = ProcessDriver (Joints,JointsInfo,jointCount)
% Body number
Joints.Driver(jointCount).Body = JointsInfo(1);
% Body position and degree constrained
Joints.Driver(jointCount).pos0 = JointsInfo(2);
Joints.Driver(jointCount).v0 = JointsInfo(3);
Joints.Driver(jointCount).a0 = JointsInfo(4);
Joints.Driver(jointCount).direction = JointsInfo(5);
end

function Joints = ProcessGround (Joints,JointsInfo,jointCount,Bodies)
% Body number
Joints.Ground(jointCount).Body = JointsInfo(1);
i = JointsInfo(1);
% Body position and degree constrained
Joints.Ground(jointCount).r0 = Bodies(i).r;
Joints.Ground(jointCount).p0 = Bodies(i).p;
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

