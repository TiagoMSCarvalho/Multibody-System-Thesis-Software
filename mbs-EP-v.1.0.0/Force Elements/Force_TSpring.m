function [forceel] = Force_TSpring(forcescount,Bodies,TSpring,Joints)
%% Initial variable definitions
%Bodies numbers
i = TSpring(forcescount).Body1;
j = TSpring(forcescount).Body2;
%Retrieve p1 and p2
p1 = Bodies(i).p;
p2 = Bodies(j).p;
%Retrieve si and sj axis from Revolute
for i = 1:Joints.Revolute
    if Joints.Revolute(i).Body1 == i && Joints.Revolute(j).Body2 == j
        sil = Joints.Revolute(i).si;
        sjl = Joints.Revolute(i).sj;
    end
end
% Force Element Constant
kt = TSpring(forcescount).Constant;
% Torsion Spring Initial 
t0 = Forces.TSpring(forcescount).theta0;
% Angle of rotation on the axis
% (e1,e2,e3) sin(phi/2) - no excel define-se o eixo da mola
%% Vector Calculus and formulation - Torsional
% Calculus of each vector displacement (new si and sj)
i1 = 6*(i-1)+1;
i2 = 6*(j-1)+1;
if t == 0
    forceel(i1:i1+5,1) = 0;
    forceel(i2:i2+5,1) = 0;
elseif t~=0
    %tem de ser feito quando tiver as variaveis e a storage dos angulos
    %feitos.
    %A ideia é:
    % O que roda j vai rodar sj, o que roda i vai rodar si, soma e calcula o
    % novo valor. (Usar os Sp's para isto) No nosso caso si = spi e sj =
    % spj.
end

end