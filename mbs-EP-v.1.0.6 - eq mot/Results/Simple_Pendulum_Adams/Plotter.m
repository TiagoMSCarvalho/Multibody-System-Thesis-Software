%Plotter

clc
clear

addpath('txt','Fig','Adams','10^8','10^7','10^6');

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
load('CoM_300_10.mat');
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
    %For the Energy Calculus
    h(1,j) = pos(2,1)*10^-3;
    m = xvel(1,j)*10^-3; 
    n = yvel(1,j)*10^-3;    
    magvel(1,j) = sqrt((m)^2 + (n)^2);
    g = 9806.65*10^-3;
    m = 11.208126;
    Energy(1,j) = (0.5*m*(magvel(1,j))^2 + m*g*1*(1+h(1,j)));
    TotalE(1,j) = m*g;
end

xpos = xpos';
ypos = ypos';
xvel = xvel';
yvel = yvel';
xaccel = xaccel';
yaccel = yaccel';

%%  Error Calc

%Position Error
%Absolute
abserrx = abs(adx-xpos);
abserry = abs(ady-ypos);
%Percentage
pererrx = (abs(adx-xpos)/abs(adx))*100;
pererry = (abs(ady-ypos)/abs(ady))*100;

%Velocity Error
%Absolute
abserrxd = abs(adxd - xvel);
abserryd = abs(adyd - yvel);
%Percentage
pererrdx = (abs(adxd-xvel)/abs(adxd))*100;
pererrdy = (abs(adyd-yvel)/abs(adyd))*100;

%Accel Error
%Absolute
abserrxdd = abs(adxdd - xaccel);
abserrydd = abs(adydd - yaccel);
%Percentage
pererrxdd = abs(adxdd - xaccel)/abs(adxdd)*100;
pererrydd = abs(adydd - yaccel)/abs(adydd)*100;



%% xplot
str = 'X Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,adx,'r');
xlabel('t[s]');
ylabel('x position [mm]');
hold on
plot(timevector,xpos,'--b','LineWidth',1.5);
legend('Adams x result','Program Results');
axis auto
hold off

%% X - Error Plot
str = 'X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrx);
xlabel('t[s]');
ylabel('Abs Diff x [mm]');
axis auto

%% X - Error Plot %
str = '% X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererrx);
xlabel('t[s]');
ylabel('Abs Diff x [%]');
axis auto

%% yplot
str = 'Y Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,ady,'r');
xlabel('t[s]');
ylabel('y position [mm]');
hold on
plot(timevector,ypos,'--b','LineWidth',1.5);
legend('Adams y result','Program Results');
axis auto
hold off

%% Y - Error Plot
str = 'Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserry);
xlabel('t[s]');
ylabel('Abs Diff y [mm]');
axis auto

%% Y - % Error Plot
str = '% Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererry);
xlabel('t[s]');
ylabel('Abs Diff y [%]');
axis auto

%% X - Vel Error Plot

str = 'Vel dx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrxd);
xlabel('t[s]');
ylabel('Abs Diff dx [mm/s]');
axis auto

%% % X - Vel Error Plot

str = 'Vel % dx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererrdx);
xlabel('t[s]');
ylabel('Abs Diff dx [%]');
axis auto

%% Y - Vel Error Plot

str = 'Vel dy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserryd);
xlabel('t[s]');
ylabel('Abs Diff dy [mm/s]');
axis auto

%% % Y - Vel Error Plot

str = 'Vel % dy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererrdy);
xlabel('t[s]');
ylabel('Abs Diff dy [%]');
axis auto

%% X - Accel Error Plot

str = 'Accel ddx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrxdd);
xlabel('t[s]');
ylabel('Abs Diff ddx [mm/s]');
axis auto

%% Y - Accel Error Plot

str = 'Accel ddy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrydd);
xlabel('t[s]');
ylabel('Abs Diff ddy [mm/s]');
axis auto

%% % X - Accel Error Plot

str = 'Accel % ddx Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererrxdd);
xlabel('t[s]');
ylabel('Abs Diff ddx [%]');
axis auto

%% % Y - Accel Error Plot

str = 'Accel % ddy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererrydd);
xlabel('t[s]');
ylabel('Abs Diff ddy [%]');
axis auto

%% X Plot Vel
str = 'X Vel Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,adxd,'r');
xlabel('t[s]');
ylabel('x velocity [mm/s]');
hold on
plot(timevector,xvel,'--b','LineWidth',1.5);
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
plot(timevector,yvel,'--b','LineWidth',1.5);
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

%% Mechanical Energy
TotalE = TotalE';
Energy = Energy';
Delta = (TotalE - Energy);

str = 'Mechanical Energy Plot';
figure('Name',str,'NumberTitle','off');

plot(timevector,Delta,'r');
xlabel('t[s]');
ylabel('E_m [J]');
axis auto
