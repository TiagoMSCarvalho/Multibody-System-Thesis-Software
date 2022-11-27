%Plotter

clc
clear

addpath('Adams_5s','wrestlength','Adams_10s');

%% Adams Info
load('Adams_timevector.mat');

%Rod displacement
load('Adams_rody.mat');

%Base displacement
load('Adams_sy.mat');


%% Program Info
load('CoM_10_500_10.mat');
load('timevector_5s.mat');

a = size(CoM,2);


for j = 1:a
    % Base pos info
    poss = CoM{1,j}(3).Position;
    sy(1,j) = poss(2,1);
    
    % Rod Info
    %Displacement
    pos = CoM{1,j}(4).Position;
    %rodx(1,j) = pos(1,1);
    rody(1,j) = pos(2,1);
    %rodz(1,j) = pos(3,1);
    
end

sy = sy';
rody = rody';

%% yplot base
str = 'Base y Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,asy,'r');
xlabel('t[s]');
ylabel('Spring Damper Support y position [m]');
hold on
plot(timevector,sy,'b');
legend('Adams y result','Program Results');
axis auto
hold off

% rodplot 

%% yplot base
str = 'Rod y Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,arody,'r');
xlabel('t[s]');
ylabel('Rod y position [m]');
hold on
plot(timevector,rody,'b');
legend('Adams y result','Program Results');
axis auto
hold off