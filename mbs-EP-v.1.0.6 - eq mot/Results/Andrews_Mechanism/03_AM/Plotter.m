%Plotter

clc
clear

addpath('Adams','03');

%% Adams Info
load('Adams_time.mat');
load('Adams_pxpos.mat');
load('Adams_pypos.mat');


%% Program Info
load('Points_03.mat');
load('Timevector_03.mat');

a = size(Points,2);

for j = 1:a
    pos = Points{1,j}(3).Position;
    pxpos(1,j) = pos(1,1);
    pypos(1,j) = pos(2,1);
end

pxpos = pxpos';
pypos = pypos';

%%  Error Calc

%Position Error
%Absolute
abserrx = abs(apxpos - pxpos);
abserry = abs(apypos - pypos);
%Percentage
pererrx = (abs(apxpos-pxpos)/abs(apxpos))*100;
pererry = (abs(apypos-pypos)/abs(apypos))*100;


%% xplot
str = 'X Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,apxpos,'r');
xlabel('t[s]');
ylabel('P x position [m]');
hold on
plot(timevector,pxpos,'--b','LineWidth',1.5);
legend('Adams x result','Program Results');
axis auto
hold off

%% X - Error Plot
str = 'X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserrx);
xlabel('t[s]');
ylabel('Err abs x [m]');
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

plot(atime,apypos,'r');
xlabel('t[s]');
ylabel('P y position [m]');
hold on
plot(timevector,pypos,'--b','LineWidth',1.5);
legend('Adams y result','Program Results');
axis auto
hold off

%% Y - Error Plot
str = 'Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,abserry);
xlabel('t[s]');
ylabel('Err abs y [m]');
axis auto

%% Y - % Error Plot
str = '% Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,pererry);
xlabel('t[s]');
ylabel('Abs Diff y [%]');
axis auto
