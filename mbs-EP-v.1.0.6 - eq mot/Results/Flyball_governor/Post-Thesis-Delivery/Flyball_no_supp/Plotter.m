%Plotter

clc
clear

%addpath('Adams','w_104');

%% Adams Info
load('Adams_timevector.mat');

%Rod displacement
load('Adams_arody.mat');
load('Adams_arodx.mat');
load('Adams_arodz.mat');

%Base displacement
%load('Adams_asy.mat');


%% Program Info
load('CoM.mat');
load('timevector.mat');

a = size(CoM,2);

for j = 1:a
    
    % Rod Info
    pos = CoM{1,j}(3).Position;
    rodx(1,j) = pos(1,1);
    rody(1,j) = pos(2,1);
    rodz(1,j) = pos(3,1);

end

rodx = rodx';
rody = rody';
rodz = rodz';

%%  Error Calc

% Rod Position Error
% Absolute
rodabserry = abs(arody-rody);
rodabserrx = abs(arodx-rodx);
rodabserrz = abs(arodz-rodz);

% Percentage
rodpererry = (abs(arody-rody))/abs(arody)*100;
rodpererrx = (abs(arodx-rodx))/abs(arodx)*100;
rodpererrz = (abs(arodz-rodz))/abs(arodz)*100;



%% ROD yplot
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
ylabel('Abs Diff y [mm]');
axis auto

%% Y - % Error Plot
str = 'Rod % Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererry);
xlabel('t[s]');
ylabel('Abs Diff y [%]');
axis auto


%% ROD xplot
str = 'Rod x Plot';
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
ylabel('Abs Diff x [mm]');
axis auto

%% X - % Error Plot
str = 'Rod % X Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrx);
xlabel('t[s]');
ylabel('Abs Diff x [%]');
axis auto


%% ROD yplot
str = 'Rod Z Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arodz,'r');
xlabel('t[s]');
ylabel('Z position [mm]');
hold on
plot(timevector,rodz,'b');
legend('Adams Z result','Program Results');
axis auto
hold off

%% Y - Error Plot
str = 'Rod Z Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodabserrz);
xlabel('t[s]');
ylabel('Abs Diff z [mm]');
axis auto

%% Y - % Error Plot
str = 'Rod % Z Error';
figure('Name',str,'NumberTitle','off');

plot(atime,rodpererrz);
xlabel('t[s]');
ylabel('Abs Diff z [%]');
axis auto

