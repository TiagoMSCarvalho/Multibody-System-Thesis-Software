%Plotter

clc
clear

addpath('Adams','Points_1010','Points_1011','Point_109');

%% Adams Info
load('Adams_timevector.mat');
load('Adams_b0xpos.mat');
load('Adams_b0ypos.mat');

% Adams Vel

load('Adams_b0xvel.mat');
load('Adams_b0yvel.mat');


ab0x = ab0xpos;
ab0y = ab0ypos;

%Adams Vel
ab0vx = ab0xvel;
ab0vy = ab0yvel;


%% Program Info
load('Points_1000.mat');
load('Timevector.mat');

a = size(Points,2);

for j = 1:a
    pos = Points{1,j}(2).Position;
    b0xpos(1,j) = pos(1,1);
    b0ypos(1,j) = pos(2,1);
    vel = Points{1,j}(2).TraVel;
    b0xvel(1,j) = vel(1,1);
    b0yvel(1,j) = vel(2,1);
end

b0xpos = b0xpos';
b0ypos = b0ypos';
b0xvel = b0xvel';
b0yvel = b0yvel';

%%  Error Calc

%Position Error
%Absolute
abserrx = abs(ab0x-b0xpos);
abserry = abs(ab0y-b0ypos);
%Percentage
pererrx = (abs(ab0x-b0xpos)/abs(ab0x))*100; %valor maximo;
pererry = (abs(ab0y-b0ypos)/abs(ab0y))*100;

%Velocity Error
%Absolute
abserrxd = abs(ab0xvel - b0xvel);
abserryd = abs(ab0yvel - b0yvel);
%Percentage
pererrdx = (abs(ab0xvel - b0xvel)/abs(ab0xvel))*100;
pererrdy = (abs(ab0yvel - b0yvel)/abs(ab0yvel))*100;



%% xplot
% str = 'X Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,ab0x,'r');
% xlabel('t[s]');
% ylabel('B_0 x position [m]');
% hold on
% plot(timevector,b0xpos,'--b','LineWidth',1.5);
% legend('Adams x result','Program Results');
% axis auto
% hold off

%% X - Error Plot
% str = 'X Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,abserrx);
% xlabel('t[s]');
% ylabel('Err abs x [m]');
% axis auto

%% X - Error Plot %
% str = '% X Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,pererrx);
% xlabel('t[s]');
% ylabel('Abs Diff x [%]');
% axis auto

%% yplot
% str = 'Y Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,ab0y,'r');
% xlabel('t[s]');
% ylabel('B_0 y position [m]');
% hold on
% plot(timevector,b0ypos,'--b','LineWidth',1.5);
% legend('Adams y result','Program Results');
% axis auto
% hold off

%% Y - Error Plot
% str = 'Y Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,abserry);
% xlabel('t[s]');
% ylabel('Err abs y [m]');
% axis auto

%% Y - % Error Plot
% str = '% Y Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,pererry);
% xlabel('t[s]');
% ylabel('Abs Diff y [%]');
% axis auto

%% X Plot Vel
% str = 'X Vel Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,ab0vx,'r');
% xlabel('t[s]');
% ylabel('B_0 x velocity [m/s]');
% hold on
% plot(timevector,b0xvel,'--b','LineWidth',1.5);
% legend('Adams x result','Program Results');
% axis auto
% hold off

%% X - Vel Error Plot

% str = 'Vel dx Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,abserrxd);
% xlabel('t[s]');
% ylabel('Err abs dx [m/s]');
% axis auto

%% % X - Vel Error Plot

% str = 'Vel % dx Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,pererrdx);
% xlabel('t[s]');
% ylabel('Abs Diff dx [%]');
% axis auto

%% Y Plot Vel
% str = 'Y Vel Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,ab0vy,'r');
% xlabel('t[s]');
% ylabel('B_0 y velocity [m/s]');
% hold on
% plot(timevector,b0yvel,'--b','LineWidth',1.5);
% legend('Adams x result','Program Results');
% axis auto
% hold off

%% Y - Vel Error Plot

str = 'Vel dy Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserryd);
xlabel('t[s]');
ylabel('Abs Diff dy [m/s]');
axis auto

%% % Y - Vel Error Plot

% str = 'Vel % dy Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,pererrdy);
% xlabel('t[s]');
% ylabel('Err abs dy [%]');
% axis auto