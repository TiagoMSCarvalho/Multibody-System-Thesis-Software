%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inputs and Outputs:
%    - fun : the fsolve tool will define value for the variables so that each
%    expression is equal to zero. The f variable carries all the values of
%    the expressions (f will be all zeros when solved)
%    - Jacobian : the Jacobian Matrix (check documentation for more
%      information)
%    - niu : right-hand-side of the velocity equations (check documentation
%      for more information)
%    - gamma : right-hand-side of the acceleration equations (check documentation
%      for more information)
%    - funCount: the equation/function counter
%    - jointCount: current Joint number on the structure Spherical
%    - Bodies: all the information of the MultiBodies System
%    - Universal: all the information of the Spherical joints present in
%    the multiBodies system
%    - Flags: 4 flags that determine which values to be calculated (each
%    flag for one of the 4 first inputs). 1 means to calculate
%
% Objective of the Function:
%    This functions objective is calculate the values of the expressions
%    the formulate each specific joints, calculate its Jacobian and
%    the right-hand-side of both velocity and acceleration equations. Each
%    calculation will only be made if the correspondent Flag value is 1.
%    Note that the multiBodies systems' Jacobian is defined with all the joints  
%    and constraints, which means, each joint just defines a few lines of
%    this matrix and the values that are calculated are only in the columns
%    correspondent to the bodies involved



function [fun,Jacobian,niu,gamma,funCount] = Joint_Spherical(fun,Jacobian,niu,gamma,funCount,jointCount,Bodies,Spherical,Flags)
%% Initial variable definitions
% Bodies numbers
i = Spherical(jointCount).Body1;
j = Spherical(jointCount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Bodies Euler Parameters
pi = Impose_Column(Bodies(i).p);
pj = Impose_Column(Bodies(j).p);
% Joint location relative to each Bodies coordinate system
spi = Impose_Column(Spherical(jointCount).spi);
spj = Impose_Column(Spherical(jointCount).spj);
%Skew Matrix 4x4
sspi = SkewMatrix4(spi);
sspj = SkewMatrix4(spj);
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
% Euler Parameters Aux Identities
Gi = Bodies(i).G;
Gj = Bodies(j).G;
Li = Bodies(i).L;
Lj = Bodies(j).L;
%Joint location in the global/absolute coordinate system
spig = Ai*spi;
spjg = Aj*spj;


%% Joint Formulation - Kinematic Problem
% Position constraint equations
if(Flags.Position == 1)
    fun(funCount:funCount+2,1) = ri + spig - rj - spjg;
end

% Jacobian Matrix
% eye command that returns identity matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 0)
    %Ci Cj aux calc tab 7.1 Nikra  (pg291)
    %Constrain defined relative to the point P central to the sph joint
    Ci = 2*(Gi*sspi + spi*pi');
    Cj = 2*(Gj*sspj + spj*pj');
    %Body i
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount:funCount+2,i1:i2)=[eye(3),Ci];
    %Body j
    i1 = 7*(j-1)+1;
    i2  = i1+6;
    Jacobian(funCount:funCount+2,i1:i2)=[-eye(3),-Cj];
end

% Right-hand-side velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+2)=0;
end

% Right-hand-side acceleration equations
if(Flags.Acceleration == 1)       
    
    %Taking the G and L derivatives out
    Gdi = Bodies(i).Gd;
    Gdj = Bodies(j).Gd;
    Ldi = Bodies(i).Ld;
    Ldj = Bodies(j).Ld;

    gamma(funCount:funCount+2) = 2*Gdj*Ldj'*spj - 2*Gdi*Ldi'*spi;
end
%% Joint Formulation - Dynamic Problem
% Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 1)
    %Ci Cj aux calc tab 7.1 Nikra  (pg291)
    %Constrain defined relative to the point P central to the sph joint
    Ci = 2*(Gi*sspi + spi*pi');
    Cj = 2*(Gj*sspj + spj*pj');
    %Body i
    i1 = 6*(i-1)+1;
    i2  = i1+5;
    Jacobian(funCount:funCount+2,i1:i2)=[eye(3),0.5*Ci*Li'];
    %Body j
    i1 = 6*(j-1)+1;
    i2  = i1+5;
    Jacobian(funCount:funCount+2,i1:i2)=[-eye(3),-0.5*Cj*Lj'];
end

if(Flags.AccelDyn == 1)
    %Body i, dynamic pre processing
    wi = Bodies(i).w;
    %Body j, dynamic pre processing
    wj = Bodies(j).w;
    %Angular Vel Skew Matrices
    swi = SkewMatrix3(wi);
    swj = SkewMatrix3(wj);
    %Derivatives of the sp's
    spid = swi*spi;
    spjd = swj*spj;
    
    gamma(funCount+2,1) = -swi*spid + swj*spjd;
end

%% Update the function counter
funCount = funCount+3;
end

