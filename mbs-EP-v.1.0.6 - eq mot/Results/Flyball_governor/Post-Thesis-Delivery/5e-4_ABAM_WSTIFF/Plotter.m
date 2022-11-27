%Plotter

clc
clear

%addpath('Adams','w_104');

%% Adams Info
load('Adams_timevector.mat');

%Rod displacement
%load('Adams_arody.mat');

%Base displacement
load('Adams_asy.mat');


%% Program Info
load('CoM.mat');
load('timevector.mat');

a = size(CoM,2);

for j = 1:a
  
    % Rod Info
    %Displacement
    %pos = CoM{1,j}(5).Position;
    %rody(1,j) = pos(2,1);
    
    % Base Info
    pos = CoM{1,j}(3).Position;
    sy(1,j) = pos(2,1);

end


%rody = rody';
sy = sy';


%%  Error Calc

% Rod Position Error
% Absolute
%rodabserry = abs(arody-rody);
baseabserry = abs(asy-sy);

% Percentage
%rodpererry = (abs(arody-rody))/abs(arody)*100;
basepererry = (abs(asy-sy))/abs(asy)*100;


%% ROD yplot
% str = 'Rod Y Plot';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,arody,'r');
% xlabel('t[s]');
% ylabel('y position [mm]');
% hold on
% plot(timevector,rody,'b');
% legend('Adams y result','Program Results');
% axis auto
% hold off
% 
% %% Y - Error Plot
% str = 'Rod Y Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,rodabserry);
% xlabel('t[s]');
% ylabel('Abs Diff y [mm]');
% axis auto
% 
% %% Y - % Error Plot
% str = 'Rod % Y Error';
% figure('Name',str,'NumberTitle','off');
% 
% plot(atime,rodpererry);
% xlabel('t[s]');
% ylabel('Abs Diff y [%]');
% axis auto


%% Base yplot
str = 'Base Y Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,asy,'r');
xlabel('t[s]');
ylabel('y position [mm]');
hold on
plot(timevector,sy,'b');
legend('Adams y result','Program Results');
axis auto
hold off

%% Y - Error Plot
str = 'Base Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,baseabserry);
xlabel('t[s]');
ylabel('Abs Diff y [mm]');
axis auto

%% Y - % Error Plot
str = 'Base % Y Error';
figure('Name',str,'NumberTitle','off');

plot(atime,basepererry);
xlabel('t[s]');
ylabel('Abs Diff y [%]');
axis auto