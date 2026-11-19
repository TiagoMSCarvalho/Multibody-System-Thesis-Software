%Plotter

clc
clear

%% Adams Info
load('Adams_timevector.mat');

%Rod displacement
load('Adams_rodx.mat');
load('Adams_rody.mat');
load('Adams_rodz.mat');

%Rod Translational Velocity
load('Adams_rodvx.mat');
load('Adams_rodvy.mat');
load('Adams_rodvz.mat');

%Rod Angular Velocity
load('Adams_basewy.mat');

%Base Angular Velocity
load('Adams_rodwy.mat');
abasewy = abasewyvel;


%% Program Info
load('CoM_Simplified.mat');
load('Timevector.mat');

a = size(CoM,2);

for j = 1:a
    % Base wy info
    basew = CoM{1,j}(1).AngVel;
    basewy(1,j) = basew(2,1);
    
    % Rod Info
    %Displacement
    pos = CoM{1,j}(3).Position;
    rodx(1,j) = pos(1,1);
    rody(1,j) = pos(2,1);
    rodz(1,j) = pos(3,1);
    %Velocity
    vel = CoM{1,j}(3).TraVel;
    rodvx(1,j) = vel(1,1);
    rodvy(1,j) = vel(2,1);
    rodvz(1,j) = vel(3,1);
    %Angular Velocity
    rodw = CoM{1,j}(3).AngVel;
    rodwy(1,j) = rodw(2,1);

end

basewy = basewy';
rodx = rodx';
rody = rody';
rodz = rodz';
rodvx = rodvx';
rodvy = rodvy';
rodvz = rodvz';
rodwy = rodwy';

%%  Error Calc

% Rod Position Error
% Absolute
rodabserrx = abs(arodx-rodx);
rodabserry = abs(arody-rody);
rodabserrz = abs(arodz-rodz);
% Percentage
rodpererrx = (abs(arodx-rodx))/abs(arodx)*100;
rodpererry = (abs(arody-rody))/abs(arody)*100;
rodpererrz = (abs(arodz-rodz))/abs(arodz)*100;

% Rod Velocity Error
% Absolute
rodabserrvx = abs(arodvx-rodvx);
rodabserrvy = abs(arodvy-rodvy);
rodabserrvz = abs(arodvz-rodvz);
% Percentage
rodpererrvx = (abs(arodvx-rodvx)/abs(arodvx))*100;
rodpererrvy = (abs(arodvy-rodvy)/abs(arodvy))*100;
rodpererrvz = (abs(arodvz-rodvz)/abs(arodvz))*100;

% Rod Angular Velocity Error
% Absolute
rodabserrwy = abs(arodwy-rodwy);
% Percentage
rodpererrwy = (abs(arodwy-rodwy))/abs(arodwy)*100;

% Base Angular Velocity Error
% Absolute
baseabserrwy = abs(abasewy-basewy);
%Percentage
basepererrwy = (abs(abasewy-basewy))/abs(abasewy)*100;

%% xplot Rod
str = 'Rod X Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arodx,'r');
xlabel('t[s]');
ylabel('x position [mm]');
hold on
plot(timevector,rodx,'b');
legend('Adams x result','Program Results');
axis auto
hold off

%% X - Error Plot
str = 'Rod X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserrx);
xlabel('t[s]');
ylabel('Err abs x [mm]');
axis auto

%% X - Error Plot %
str = 'Rod % X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrx);
xlabel('t[s]');
ylabel('Err abs x [%]');
axis auto

%% yplot
str = 'Rod Y Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arody,'r');
xlabel('t[s]');
ylabel('y position [mm]');
hold on
plot(timevector,rody,'b');
legend('Adams y result','Program Results');
axis auto
hold off

%% Y - Error Plot
str = 'Rod Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserry);
xlabel('t[s]');
ylabel('Err abs y [mm]');
axis auto

%% Y - % Error Plot
str = 'Rod % Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererry);
xlabel('t[s]');
ylabel('Err abs y [%]');
axis auto

%% zplot
str = 'Rod Z Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arodz,'r');
xlabel('t[s]');
ylabel('z position [mm]');
hold on
plot(timevector,rodz,'b');
legend('Adams z result','Program Results');
axis auto
hold off

%% Z - Error Plot
str = 'Rod Z Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserrz);
xlabel('t[s]');
ylabel('Err abs z [mm]');
axis auto

%% Z - % Error Plot
str = 'Rod % Z Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrz);
xlabel('t[s]');
ylabel('Err abs z [%]');
axis auto
%% Rod X - Vel Plot
str = 'Rod Vx Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arodvx,'r');
xlabel('t[s]');
ylabel('Vx velocity [mm/s]');
hold on
plot(timevector,rodvx,'b');
legend('Adams Vx result','Program Results');
axis auto
hold off

%% Rod X - Vel Error Plot
str = 'Vel Vx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserrvx);
xlabel('t[s]');
ylabel('Err abs Vx [mm/s]');
axis auto

%% Rod % X - Vel Error Plot

str = 'Vel % Vx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrvx);
xlabel('t[s]');
ylabel('Err abs Vx [%]');
axis auto

%% Rod Y - Vel Plot
str = 'Rod Vy Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arodvy,'r');
xlabel('t[s]');
ylabel('Vy velocity [mm/s]');
hold on
plot(timevector,rodvy,'b');
legend('Adams Vy result','Program Results');
axis auto
hold off

%% Rod Y - Vel Error Plot

str = 'Vel Vy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserrvy);
xlabel('t[s]');
ylabel('Err abs Vy [mm/s]');
axis auto

%% Rod % Y - Vel Error Plot

str = 'Vel % Vy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrvy);
xlabel('t[s]');
ylabel('Err abs Vy [%]');
axis auto

%% Rod Z - Vel Plot
str = 'Rod Vz Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arodvz,'r');
xlabel('t[s]');
ylabel('Vz velocity [mm/s]');
hold on
plot(timevector,rodvz,'b');
legend('Adams Vz result','Program Results');
axis auto
hold off

%% Rod Z - Vel Error Plot

str = 'Vel Vz Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserrvz);
xlabel('t[s]');
ylabel('Err abs Vz [mm/s]');
axis auto

%% Rod % Y - Vel Error Plot

str = 'Vel % Vz Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrvz);
xlabel('t[s]');
ylabel('Err abs Vz [%]');
axis auto

%% Rod Wy - Vel Plot
str = 'Rod wy Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arodwy,'r');
xlabel('t[s]');
ylabel('wy velocity [rad/s]');
hold on
plot(timevector,rodwy,'b');
legend('Adams wy result','Program Results');
axis auto
hold off

%% Rod Wy - Vel Error Plot

str = 'Rod wy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserrwy);
xlabel('t[s]');
ylabel('Err abs wy [rad/s]');
axis auto

%% Rod % Wy - Vel Error Plot

str = 'Rod % wy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrwy);
xlabel('t[s]');
ylabel('Err abs Wy [%]');
axis auto

%% Base Wy - Vel Plot
str = 'Base wy Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,abasewy,'r');
xlabel('t[s]');
ylabel('wy velocity [rad/s]');
hold on
plot(timevector,basewy,'b');
legend('Adams Wy result','Program Results');
axis auto
hold off

%% Base Wy - Vel Error Plot

str = 'Base wy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,baseabserrwy);
xlabel('t[s]');
ylabel('Err abs wy [mm/s]');
axis auto

%% Base % Wy - Vel Error Plot

str = 'Base % wy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,basepererrwy);
xlabel('t[s]');
ylabel('Err abs wy [%]');
axis auto