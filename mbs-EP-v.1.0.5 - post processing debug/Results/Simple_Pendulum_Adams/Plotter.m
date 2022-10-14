%Plotter

clc
clear

%% Adams Info
load('Adams_timevector.mat');
load('Adams_xvalues.mat');
load('Adams_yvalues.mat');

% Adams Vel

load('Adams_xvel.mat');
load('Adams_yvel.mat');

% Adams Accel
load('Adams_xaccel.mat');
load('Adams_yaccel.mat');

adx = Adams_xvalues;
ady = Adams_yvalues;
atime = adamstime;

%Adams Vel
adxd = Adams_xvel;
adyd = Adams_yvel;

%Adams Accel
adxdd = Adams_xaccel;
adydd = Adams_yaccel;

%% Program Info
load('CoM_postdebug1.mat');
load('Timevector.mat');

a = size(CoM,2);

for j = 1:a
    pos = CoM{1,j}(3).Position;
    xpos(1,j) = pos(1,1);
    ypos(1,j) = pos(2,1);
    vel = CoM{1,j}(3).TraVel;
    xvel(1,j) = vel(1,1);
    yvel(1,j) = vel(2,1);
    accel = CoM{1,j}(3).TraAccel;
    xaccel(1,j) = accel(1,1);
    yaccel(1,j) =  accel(2,1);
end

xpos = xpos';
ypos = ypos';
xvel = xvel';
yvel = yvel';
xaccel = xaccel';
yaccel = yaccel';

%%  Error Calc

abserrx = abs(adx-xpos);
abserry = abs(ady-ypos);

%Velocity Error
abserrxd = abs(adxd - xvel);
abserryd = abs(adyd - yvel);

%Accel Error
abserrxdd = abs(adxdd - xaccel);
abserrydd = abs(adydd - yaccel);


%% xplot
str = 'X Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,adx,'r');
xlabel('t[s]');
ylabel('x position [mm]');
hold on
plot(timevector,xpos,'b');
legend('Adams x result','Program Results');
axis auto
hold off

%% X - Error Plot
str = 'X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrx);
xlabel('t[s]');
ylabel('Err abs x [mm]');
axis auto

%% yplot
str = 'Y Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,ady,'r');
xlabel('t[s]');
ylabel('y position [mm]');
hold on
plot(timevector,ypos,'b');
legend('Adams y result','Program Results');
axis auto
hold off

%% Y - Error Plot
str = 'Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserry);
xlabel('t[s]');
ylabel('Err abs y [mm]');
axis auto

%% X - Vel Error Plot

str = 'Vel dx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrxd);
xlabel('t[s]');
ylabel('Err abs dx [mm/s]');
axis auto

%% Y - Vel Error Plot

str = 'Vel dy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserryd);
xlabel('t[s]');
ylabel('Err abs dy [mm/s]');
axis auto

%% X - Accel Error Plot

str = 'Accel ddx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrxdd);
xlabel('t[s]');
ylabel('Err abs ddx [mm/s]');
axis auto

%% Y - Accel Error Plot

str = 'Accel ddy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrydd);
xlabel('t[s]');
ylabel('Err abs ddy [mm/s]');
axis auto

%% X Plot Vel
str = 'X Vel Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,adxd,'r');
xlabel('t[s]');
ylabel('x velocity [mm/s]');
hold on
plot(timevector,xvel,'b');
legend('Adams x result','Program Results');
axis auto
hold off

%% Y Plot Vel
str = 'Y Vel Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,adyd,'r');
xlabel('t[s]');
ylabel('y velocity [mm/s]');
hold on
plot(timevector,yvel,'b');
legend('Adams x result','Program Results');
axis auto
hold off

%% X Plot Accel
str = 'X Accel Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,adxdd,'r');
xlabel('t[s]');
ylabel('x acceleration [mm/s^2]');
hold on
plot(timevector,xaccel,'b');
legend('Adams x result','Program Results');
axis auto
hold off

%% Y Plot Accel
str = 'Y Accel Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,adydd,'r');
xlabel('t[s]');
ylabel('y acceleration [mm/s^2]');
hold on
plot(timevector,yaccel,'b');
legend('Adams x result','Program Results');
axis auto
hold off