function [forceel] = Force_TSpring(forcescount,Bodies,TSpring,t)
%% Initial variable definitions
%Bodies numbers
i = TSpring(forcescount).Body1;
j = TSpring(forcescount).Body2;
% Force Element Constant
kt = TSpring(forcescount).Constant;
% Torsion Spring Initial 
t0 = Forces.TSpring(forcescount).theta0;
% Initial Vectors that define the initial angle
si = Forces.TSpring(focescount).si;
sj = Forces.TSpring(forcescount).sj;
%Bodies Velocities
wi = Bodies(i).w;
wj = Bodies(j).w;
%Initial Bodies Velocities
wi0 = Bodies(i).w0;
wj0 = Bodies(j).w0;
%% Vector Calculus and formulation - Torsional
% Calculus of each vector displacement (new si and sj)
i1 = 6*(i-1)+1;
i2 = 6*(j-1)+1;
if t == 0
    forceel(i1:i1+5,1) = 0;
    forceel(i2:i2+5,1) = 0;
elseif t~=0
    %tem de ser feito quando tier as variaveis e a storage dos angulos
    %feitos.
    %A ideia é:
    % O que roda j vai rodar sj, o que roda i vai rodar i, soma e calcula o
    % novo valor.
end

end