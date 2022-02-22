%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inputs and Outputs:
%    - fun : the fsolve tool wi1l define value for the variables so that each
%    expression is equal to zero. The f variable carries all the values of
%    the expressions (f wi1l be all zeros when solved)
%    - Jacobian : the Jacobian Matrix (check documentation for more
%      information)
%    - niu : right-hand-side of the velocity equations (check documentation
%      for more information)
%    - gamma : right-hand-side of the acceleration equations (check documentation
%      for more information)
%    - funCount: the equation/function counter
%    - jointCount: current Joint number on the structure Revolute
%    - Bodies: all the information of the MultiBodies System
%    - Cylindrical: all the information of the Cylindrical joints present in
%    the multiBodies system
%    - Flags: 4 flags that determine which values to be calculated (each
%    flag for one of the 4 first inputs). 1 means to calculate
%
% Objective of the Function:
%    This functions objective is calculate the values of the expressions
%    the formulate each specific joints, calculate its Jacobian and
%    the right-hand-side of both velocity and acceleration equations. Each
%    calculation wi1l only be made if the correspondent Flag value is 1.
%    Note that the multiBodies systems' Jacobian is defined with all the joints  
%    and constraints, which means, each joint just defines a few lines of
%    this matrix and the values that are calculated are only in the columns
%    correspondent to the bodies involved

function [fun,Jacobian,niu,gamma,funCount] = Joint_Cylindrical(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Cylindrical,Flags)
% Function to evaluate all quantities required by the revolute joint
% Define the body numbers
i = Cylindrical(jointCount).Body1;
j = Cylindrical(jointCount).Body2;
%Bodie Position Vector
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Bodies Euler Parameters
pi = Impose_Column(Bodies(i).p);
pj = Impose_Column(Bodies(j).p);
%Body Vector for P 
spi = Impose_Column(Cylindrical(jointCount).spi);
spj = Impose_Column(Cylindrical(jointCount).spj);
%Vector in the body
si = Impose_Column(Cylindrical(jointCount).si);
sj = Impose_Column(Cylindrical(jointCount).sj);
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
%Vector si and sj in the global frame
sig = Ai*si;
sjg = Aj*sj;
%Skew Matrix body
ssi = SkewMatrix4(si);
ssj = SkewMatrix4(sj);
%Skew Matrix Global
ssig = SkewMatrix4(sig);
ssjg = SkewMatrix4(sjg);
%SkewMatrix Vector si and sj for P
sspi = SkewMatrix4(spi);
sspj = SkewMatrix4(spj);
% Euler Parameters Aux Identities
Gi = Bodies(i).G;
Gj = Bodies(j).G;
Li = Bodies(i).L;
Lj = Bodies(j).L;

%Vector between P's of the Bodies, di is already global
d = -ri - Ai*spi + rj + Aj*spj; %Ai, Aj were in different positions
%Skew vector d
sd = SkewMatrix4(d);
% 2 non-colinear vector that are perpendicular to the vector si
% si_p1 = [-si(2); si(1); 0];
% si_p2 = cross(si,si_p1);
[qi,ti] = PerpendicularVectors(si);
%Perpendicular vectors in the global frame
qig = Ai*qi;
tig = Ai*ti;
%SkewMatrix of the perpendicular vectors body frame
sqi = SkewMatrix4(qi);
sti = SkewMatrix4(ti);

% Form the position constraint equations
if(Flags.Position == 1)
    fun(funCount,1) = qig'*d;
    fun(funCount+1,1) = tig'*d;
    fun(funCount+2,1) = qig'*sjg;
    fun(funCount+3,1) = tig'*sjg;
end

% Form the Jacobian Matrix
if (Flags.Jacobian == 1)
    %For this contraint the two parallel equations are modified to be 2
    %perpendicular constrains:
        %Type 2 - In relation with the vector d between Pi and Pj so Bi and
            %Bj wi1l be defined in relation to Pi and Pj, Ci will be
            %defined to qi and ti
        %Type 1 - Defined as the Revolute joint, q and t are 2
        %perpendicular vectors to si and the constrain is done in relation
        %to sj to allow rotation just around 1 axis so we need:
            %Ciq in rel to q, Cit in rel to t and Csj since the
            %perpendicularity is in rel to sj.
    Bi = 2*(Gi*sspi + spi*pi');
    Bj = 2*(Gj*sspj + spj*pj');
    Ciq = 2*(Gi*sqi + qi*pi');
    Cit = 2*(Gi*sti + ti*pi');
    Cjs = 2*(Gj*ssj + sj*pj');
    %Body i
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount,i1:i2) = [-qig',-qig'*Bi+d'*Ciq];
    Jacobian(funCount+1,i1:i2) = [-tig',-tig'*Bi+d'*Cit];
    Jacobian(funCount+2,i1:i2) = [0,0,0,sjg'*Ciq]; %15-02 minus added
    Jacobian(funCount+3,i1:i2) = [0,0,0,sjg'*Cit]; %15-02 minus added
    %Body j
    i1 = 7*(j-1)+1;
    i2 = i1+6;
    Jacobian(funCount,i1:i2) = [qig',qig'*Bj];
    Jacobian(funCount+1,i1:i2) = [tig',tig'*Bj]; % Diff in Wichita but following Nikra Logic and equation pg 201
    Jacobian(funCount+2,i1:i2) = [0,0,0,qig'*Cjs];
    Jacobian(funCount+3,i1:i2) = [0,0,0,tig'*Cjs];
end

% Form the r.h.s velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+3) = 0;
end

% Form the r.h.s. acceleration equations
if(Flags.Acceleration == 1)
    
    %Taking the G and L derivatives out
    Gdi = Bodies(i).Gd;
    Gdj = Bodies(j).Gd;
    Ldi = Bodies(i).Ld;
    Ldj = Bodies(j).Ld;
    %Extract the angular velocity vectors from the Bodies Struct
    wgi = Bodies(i).wg;
    wli = Bodies(i).wl;
    wgj = Bodies(j).wg;
    wlj = Bodies(j).wl;
    %Derivatives of qi,ti and sj in the global frame
    qid = SkewMatrix3(wgi)*qig;
    tid = SkewMatrix3(wgi)*tig;
    sjd = SkewMatrix3(wgj)*sjg;
    %For the derivative of d we have to use the eq defined above in the
    %code d = ri + Ai*spi - rj - Aj*spj;
    rjd = SkewMatrix3(wgj)*rj;  
    rid = SkewMatrix3(wgi)*ri;
    spid = SkewMatrix3(wgi)*ri;
    spjd = SkewMatrix3(wgj)*rj;
    dd = - rid - Ai*SkewMatrix3(wgi)*spi + rjd + Aj*SkewMatrix3(wgj)*spj; %ri Ai e rj Aj estavam trocados.
    
    gamma(funCount) = qig'*(-2*Gdj*Ldj'*spjd - (-2*Gdi*Ldi'*spid)) + d'*(-2*Gdi*Ldi'*qi) - 2*dd'*qid; %15-02 + added since h = -2;
    gamma(funCount+1) = tig'*(-2*Gdj*Ldj'*spjd - (-2*Gdi*Ldi'*spid)) + d'*(-2*Gdi*Ldi'*ti) - 2*dd'*tid; %15-02 + added since h = -2;
    gamma(funCount+2) = qig'*(-2*Gdj*Ldj'*sj) + sjg'*(-2*Gdi*Ldi'*qi) - 2*qid'*sjd;
    gamma(funCount+3) = tig'*(-2*Gdj*Ldj'*sj) + sjg'*(-2*Gdi*Ldi'*ti) - 2*tid'*sjd;    

    %Type 2 - Following the Jacobian logic the hi and hj wi1l be defined in relation
    %to Pi and Pj and D*hi wi1l be defined in relation to qi and wi1l follow the 7.2 gamma equations.
    %Type 1 - hi relative to the q or t vector and hj relative to sj
    
end
   
% Update the line counter
funCount = funCount+4;
end