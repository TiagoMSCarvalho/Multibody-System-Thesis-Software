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
%    - Translation: all the information of the Translation joints present in
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

function [fun,Jacobian,niu,gamma,funCount] = Joint_Translation(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Translation,Flags)
%% Initial variable definitions
% Define the Bodies numbers
i = Translation(jointCount).Body1;
j = Translation(jointCount).Body2;
% Bodies Position Vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Bodies Euler Parameters
pi = Impose_Column(Bodies(i).p);
pj = Impose_Column(Bodies(j).p);
%Joint Location relative to each bodie frame
spi = Impose_Column(Translation(jointCount).spi);
spj = Impose_Column(Translation(jointCount).spj);
%Definition of the vectors in body coord system
si = Impose_Column(Translation(jointCount).si);
sj = Impose_Column(Translation(jointCount).sj);
%Rotation Matrix
Ai = Bodies(i).A;
Aj = Bodies(j).A;
%Vector sj in the global frame
sjg = Aj*sj;
%Skew Matrix body
ssj = SkewMatrix4(sj);
%SkewMatrix Vector si and sj for P
sspi = SkewMatrix4(spi);
sspj = SkewMatrix4(spj);
% Euler Parameters Aux Identities
Gi = Bodies(i).G;
Gj = Bodies(j).G;
Li = Bodies(i).L;
Lj = Bodies(j).L;

%Vector between P's of the Bodies
d = rj + Aj*spj - ri - Ai*spi;

% 2 non-colinear vector that are perpendicular to the vector si
% si_p1 = [-si(2); si(1); 0];
% si_p2 = cross(si,si_p1);
[qi,ti] = PerpendicularVectors(si);
[qj,~] = PerpendicularVectors(sj);
%Perpendicular vectors in the global frame
%Body i
qig = Ai*qi;
tig = Ai*ti;
%Body j
qjg = Aj*qj;
%SkewMatrix of the perpendicular vectors body frame
%Body i
sqi = SkewMatrix4(qi);
sti = SkewMatrix4(ti);
%Body j
sqj = SkewMatrix4(qj);
%% Joint Formulation - Kinematic Problem
% Form the position constraint equations
if(Flags.Position == 1)
    fun(funCount,1)   = qig'*d;
    fun(funCount+1,1) = tig'*d;
    fun(funCount+2,1) = qig'*sjg;
    fun(funCount+3,1) = tig'*sjg;
    fun(funCount+4,1) = tig'*qjg;
end

% Form the Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 0)
    %The translational constraint is constructed with the equations from
    %the cylindrical constraint however rotation is not allowed and the 3rd
    %dof of rotation should be contrained. For this we need to define a
    %vector qj perpendicular to sj in body j, this vector is set to be
    %perpendicular to ti (constrain type 1) from the body i, this wi1l lock
    %the bodies together around the axis, making impossible for them to
    %rotate without being a rigid body.
    %Cyl equations:
        %Type 2 - In relation with the vector d between Pi and Pj so Bi and
            %Bj wi1l be defined in relation to Pi and Pj
        %Type 1 - Defined as the Revolute joint, q and t are 2
        %perpendicular vectors to si and the constrain is done in relation
        %to sj to allow rotation just around 1 axis so we need:
            %Ciq in rel to q, Cit in rel to t and Csj since the
            %perpendicularity is in rel to sj.
    %Translation equation:
        %Since the constrain is define between qj and ti the Ci wi1l be
        %defined in relation to ti and the Cj wi1l be defined in relation
        %to qj, thus we have to create a new aux: Cjq
    %Aux C's:
        %C's usually are within the body frame, the Cq is usually within
        %the global frame.
    Bi = 2*(Gi*sspi + spi*pi');
    Bj = 2*(Gj*sspj + spj*pj');
    Ciq = 2*(Gi*sqi + qi*pi');
    Cit = 2*(Gi*sti + ti*pi');
    Cjs = 2*(Gj*ssj + sj*pj');
    Cjq = 2*(Gj*sqj + qj*pj');
    %Body i
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [-qig',-qig'*Bi+d'*Ciq];
    Jacobian(funCount+1,i1:i2) = [-tig',-tig'*Bi+d'*Cit];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,sjg'*Ciq];
    Jacobian(funCount+3,i1:i2) = [0,0,0,sjg'*Cit];
    Jacobian(funCount+4,i1:i2) = [0,0,0,qjg'*Cit];
    %Body j
    i1 = 7*(j-1)+1;
    i2 = i1+6;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [qig',qig'*Bj];
    Jacobian(funCount+1,i1:i2) = [tig',tig'*Bj];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,qig'*Cjs];
    Jacobian(funCount+3,i1:i2) = [0,0,0,tig'*Cjs];
    Jacobian(funCount+4,i1:i2) = [0,0,0,tig'*Cjq];
end

% Form the r.h.s velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+4) = 0;
end


% Form the r.h.s. acceleration equations
if(Flags.Acceleration == 1)    
    
    %Taking the G and L derivatives out
    Gdi = Bodies(i).Gd;
    Gdj = Bodies(j).Gd;
    Ldi = Bodies(i).Ld;
    Ldj = Bodies(j).Ld;
    %Extract the angular velocity vectors from the Bodies Struct
    %wgi = Bodies(i).wg;
    %wgj = Bodies(j).wg;
    wli = Bodies(i).wl;
    wlj = Bodies(j).wl;
    %Derivatives of qi,ti and sj in the global frame
    qid = Ai*SkewMatrix3(wli)*qi;
    tid = Ai*SkewMatrix3(wli)*ti;
    sjd = Aj*SkewMatrix3(wlj)*sj;
    qjd = Aj*SkewMatrix3(wlj)*qj;
    %For the derivative of d we have to use the eq defined above in the
    %code d = rj + Aj*spj - ri - Ai*spi;
    rid = Bodies(i).rd;
    rjd = Bodies(j).rd;
    dd =  rjd + Aj*SkewMatrix3(wlj)*spj - rid - Ai*SkewMatrix3(wli)*spi;
    %Spid and Spjd for the 2nd perpendiculars
    spid = SkewMatrix3(wli)*spi;
    spjd = SkewMatrix3(wlj)*spj;
    
    gamma(funCount) = qig'*(-2*Gdj*Ldj'*spjd + 2*Gdi*Ldi'*spid) + d'*(-2*Gdi*Ldi'*qi) - 2*dd'*qid; 
    gamma(funCount+1) = tig'*(-2*Gdj*Ldj'*spjd + 2*Gdi*Ldi'*spid) + d'*(-2*Gdi*Ldi'*ti) - 2*dd'*tid;
    gamma(funCount+2) = qig'*(-2*Gdj*Ldj'*sj) + sjg'*(-2*Gdi*Ldi'*qi) - 2*qid'*sjd; 
    gamma(funCount+3) = tig'*(-2*Gdj*Ldj'*sj) + sjg'*(-2*Gdi*Ldi'*ti) - 2*tid'*sjd;    
    gamma(funCount+4) = tig'*(-2*Gdj*Ldj'*qj) + qjg'*(-2*Gdi*Ldi'*ti) - 2*tid'*qjd;

    %Type 2 - Following the Jacobian logic the hi and hj wi1l be defined in relation
    %to Pi and Pj and D*hi wi1l be defined in relation to qi and wi1l follow the 7.2 gamma equations.
    %Type 1 - hi relative to the q or t vector and hj relative to sj
    %Type 1 last eq - we have ti and qj so following the logic hj is
    %defined relative to qj and hi relative to ti;
end
%% Joint Formulation - Dynamic Problem

% Form the Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 1)
    Bi = 2*(Gi*sspi + spi*pi');
    Bj = 2*(Gj*sspj + spj*pj');
    Ciq = 2*(Gi*sqi + qi*pi');
    Cit = 2*(Gi*sti + ti*pi');
    Cjs = 2*(Gj*ssj + sj*pj');
    Cjq = 2*(Gj*sqj + qj*pj');
    %Body i
    i1 = 6*(i-1)+1;
    i2  = i1+5;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [-qig',0.5*(-qig'*Bi+d'*Ciq)*Li'];
    Jacobian(funCount+1,i1:i2) = [-tig',0.5*(-tig'*Bi+d'*Cit)*Li'];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,0.5*(sjg'*Ciq)*Li'];
    Jacobian(funCount+3,i1:i2) = [0,0,0,0.5*(sjg'*Cit)*Li'];
    Jacobian(funCount+4,i1:i2) = [0,0,0,0.5*(qjg'*Cit)*Li'];
    %Body j
    i1 = 6*(j-1)+1;
    i2 = i1+5;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [qig',0.5*(qig'*Bj)*Lj'];
    Jacobian(funCount+1,i1:i2) = [tig',0.5*(tig'*Bj)*Lj'];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,0.5*(qig'*Cjs)*Lj'];
    Jacobian(funCount+3,i1:i2) = [0,0,0,0.5*(tig'*Cjs)*Lj'];
    Jacobian(funCount+4,i1:i2) = [0,0,0,0.5*(tig'*Cjq)*Lj'];
end

if(Flags.AccelDyn == 1)
    %Body i, dynamic pre processing
    rdi = Bodies(i).rd;
    wi = Bodies(i).w;
    %Body j, dynamic pre processing
    rdj = Bodies(j).rd;
    wj = Bodies(j).w;
    %Angular Vel Skew Matrices
    swi = SkewMatrix3(wi);
    swj = SkewMatrix3(wj);
    %Derivatives of the sp's
    spid = swi*spi;
    spjd = swj*spj;
    %Derivatives of qi,ti and sj in the global frame
    qid = swi*qi;
    tid = swi*ti;
    sjd = swj*sj;
    qjd = swj*qj;
    
    % d vector derivative -> A was taken out because the velocities given
    % are in the absolute frame
    dd = rdj + swj*spj - rdi - swi*spi;
    dd = Impose_Column(dd);
    
    gamma(funCount,1) = -2*dd'*qid - d'*swi*qid + qi'*(swi*spid - swj*spjd);
    gamma(funCount+1,1) = -2*dd'*tid - d'*swi*tid + ti'*(swi*spid - swj*spjd);
    gamma(funCount+2,1) = -2*qid'*sjd + qid'*swi*sj + sjd'*swj*qi;
    gamma(funCount+3,1) = -2*tid'*sjd + tid'*swi*sj + sjd'*swj*ti;
    gamma(funCount+4,1) = -2*tid'*qjd + tid'*swi*qj + qjd'*swj*ti;
end

%% Update the line counter
funCount = funCount+5;
end