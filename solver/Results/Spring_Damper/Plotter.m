%Plotter

clc
clear

addpath('Adams','Matlab');

%% Adams Info
load('Adams_timevector.mat');
load('Adams_Spring_ypos.mat');


%% Program Info
load('CoM_Spring.mat');
load('timevector.mat');

a = size(CoM,2);

for j = 1:a
    pos = CoM{1,j}(2).Position;
    mypos(j,1) = pos(2,1);
end



%% xplot
str = 'Y Plot';
figure('Name',str,'NumberTitle','off');

plot(atime,ypos,'r');
xlabel('t[s]');
ylabel('y position [m]');
hold on
plot(timevector,mypos,'--b','LineWidth',1.5);
legend('Adams y result','Program Results');
axis auto
hold off
