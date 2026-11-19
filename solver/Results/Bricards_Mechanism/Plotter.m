%Plotter

clc
clear

addpath('Adams','Points_108','Points_107','Points_106');

%% Adams Info
load('Adams_timevector.mat');
load('Adams_p3xpos.mat');
load('Adams_p3ypos.mat');
load('Adams_p3zpos.mat');

ap3xpos = P3xpos;
ap3ypos = P3ypos;
ap3zpos = P3zpos;

% % Adams Vel
% 
% load('Adams_b0xvel.mat');
% load('Adams_b0yvel.mat');
% 
% 
% ab0x = ab0xpos;
% ab0y = ab0ypos;
% 
% %Adams Vel
% ab0vx = ab0xvel;
% ab0vy = ab0yvel;


%% Program Info
load('Points_500106.mat');
load('Timevector.mat');

a = size(Points,2);

for j = 1:a
    pos = Points{1,j}(4).Position;
    p3xpos(1,j) = pos(1,1);
    p3ypos(1,j) = pos(2,1);
    p3zpos(1,j) = pos(3,1);
%     vel = Points{1,j}(2).TraVel;
%     p3xvel(1,j) = vel(1,1);
%     p3yvel(1,j) = vel(2,1);
end

p3xpos = p3xpos';
p3ypos = p3ypos';
p3zpos = p3zpos';

%%  Error Calc

%Position Error
%Absolute
abserrx = abs(ap3xpos - p3xpos);
abserry = abs(ap3ypos - p3ypos);
abserrz = abs(ap3zpos - p3zpos);
%Percentage
pererrx = (abs(ap3xpos-p3xpos)/abs(ap3xpos))*100;
pererry = (abs(ap3ypos-p3ypos)/abs(ap3ypos))*100;
pererrz = (abs(ap3zpos-p3zpos)/abs(ap3zpos))*100;

%Velocity Error
%Absolute
% abserrxd = abs(ab0xvel - b0xvel);
% abserryd = abs(ab0yvel - b0yvel);
%Percentage
% pererrdx = (abs(ab0xvel - b0xvel)/abs(ab0xvel))*100;
% pererrdy = (abs(ab0yvel - b0yvel)/abs(ab0yvel))*100;



%% xplot
% str = 'X Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,ap3xpos,'r');
% xlabel('t[s]');
% ylabel('P_3 x position [m]');
% hold on
% plot(timevector,p3xpos,'--b','LineWidth',1.5);
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
str = '% X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererrx);
xlabel('t[s]');
ylabel('Abs Diff x [%]');
axis auto

%% yplot
% str = 'Y Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,ap3ypos,'r');
% xlabel('t[s]');
% ylabel('P_3 y position [m]');
% hold on
% plot(timevector,p3ypos,'--b','LineWidth',1.5);
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
str = '% Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererry);
xlabel('t[s]');
ylabel('Abs Diff y [%]');
axis auto

%% zplot
% str = 'Z Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,ap3zpos,'r');
% xlabel('t[s]');
% ylabel('P_3 z position [m]');
% hold on
% plot(timevector,p3zpos,'--b','LineWidth',1.5);
% legend('Adams z result','Program Results');
% axis auto
% hold off

%% Z - Error Plot
% str = 'Y Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,abserrz);
% xlabel('t[s]');
% ylabel('Err abs z [mm]');
% axis auto

%% Z - % Error Plot
str = '% Z Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererry);
xlabel('t[s]');
ylabel('Abs Diff z [%]');
axis auto

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
% 
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
% ylabel('Err abs dx [%]');
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

% str = 'Vel dy Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,abserryd);
% xlabel('t[s]');
% ylabel('Err abs dy [m/s]');
% axis auto

%% % Y - Vel Error Plot

% str = 'Vel % dy Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,pererrdy);
% xlabel('t[s]');
% ylabel('Err abs dy [%]');
% axis auto