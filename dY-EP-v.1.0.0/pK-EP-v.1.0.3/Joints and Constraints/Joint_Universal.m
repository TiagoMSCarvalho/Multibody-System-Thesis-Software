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
%    - jointCount: current Joint number on the structure Universal
%    - Bodies: all the information of the MultiBodies System
%    - Universal: all the information of the Universal joints present in
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


function [fun,Jacobian,niu,gamma,funCount] = Joint_Universal(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Universal,Flags)
%% Initial variable definitions
% Bodies numbers
i = Universal(jointCount).Body1;
j = Universal(jointCount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Bodies Euler Parameters
pi = Impose_Column(Bodies(i).p);
pj = Impose_Column(Bodies(j).p);
% Joint location relative to each Bodies coordinate system
spi = Impose_Column(Universal(jointCount).spi);
spj = Impose_Column(Universal(jointCount).spj);
% Definition of the vectors (each one in one of the bodies coordinate
% system) that should be perpendicular
si = Impose_Column(Universal(jointCount).si);
sj = Impose_Column(Universal(jointCount).sj);
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
%Vector si and sj in the global frame
sig = Ai*si;
sjg = Aj*sj;
%Vector sip and sjp in the global frame
spig = Ai*spi;
spjg = Aj*spj;
%SkewMatrix Vector
ssi= SkewMatrix4(si);
ssj= SkewMatrix4(sj);
%SkewMatrix Vector si and sj for P
sspi = SkewMatrix4(spi);
sspj = SkewMatrix4(spj);
% Euler Parameters Aux Identities
Gi = Bodies(i).G;
Gj = Bodies(j).G;


%% Joint Formulation
% Position constraint equations
if(Flags.Position == 1)
    fun(funCount:funCount+2,1) = ri + spig - rj - spjg;
    fun(funCount+3,1) = sig'*sjg;
end

% Jacobian Matrix
%Universal 2ª eq é referente ao si e ao sj e nao ao spi e spj normal.
if (Flags.Jacobian == 1)
    %Ci Cj written to spi and spj aux calc tab 7.1 Nikra  (pg201)
    Cpi = 2*(Gi*sspi + spi*pi');
    Cpj = 2*(Gj*sspj + spj*pj');
    %Ci Cj written to si and sj
    Ci = 2*(Gi*ssi + si*pi');
    Cj = 2*(Gj*ssj + sj*pj');
    %Body i
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount:funCount+2,i1:i2)=[eye(3), Cpi];
    Jacobian(funCount+3,i1:i2)=[0,0,0,sjg'*Ci];
    %Body j
    i1 = 7*(j-1)+1;
    i2 = i1+6;
    Jacobian(funCount:funCount+2,i1:i2)=[-eye(3),-Cpj];
    Jacobian(funCount+3,i1:i2)=[0,0,0,sig'*Cj];
end

% Right-hand-side velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+3) = 0;
end

% Right-hand-side acceleration equations
if(Flags.Acceleration == 1)
    
    %Taking the G and L derivatives out
    Gdi = Bodies(i).Gd;
    Gdj = Bodies(j).Gd;
    Ldi = Bodies(i).Ld;
    Ldj = Bodies(j).Ld;
    %Extract the angular velocity vectors from the Bodies Struct
    wli = Bodies(i).wl;
    wlj = Bodies(j).wl;
    %Derivatives of si and sj in the global frame
    sid = Ai*SkewMatrix3(wli)*si;
    sjd = Aj*SkewMatrix3(wlj)*sj;
    
    gamma(funCount:funCount+2) = 2*Gdj*Ldj.'*spj - 2*Gdi*Ldi.'*spi;
    gamma(funCount+3) = sig'*(-2*Gdj*Ldj.'*sj) + sjg'*(-2*Gdi*Ldi.'*si) - 2*sid'*sjd;
end
   
% Update the function counter
funCount = funCount+4;
end
