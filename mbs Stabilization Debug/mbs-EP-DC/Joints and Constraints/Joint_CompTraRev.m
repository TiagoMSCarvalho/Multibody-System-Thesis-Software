function [fun,Jacobian,niu,gamma,funCount] = Joint_CompTraRev(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, TraRev,Flags)
%Composite joint with a Revolute and Prismatic joint.
% si -> REV axis; sj -> PRI axis
% Equations :
    % n2,1 : si'*d
    % n2,1 : qi'*d
    % n1,1 : qi'*qj
    % n1,1 : si'*sj

%% Initial variable definitions
% Define the Bodies numbers
i = TraRev(jointCount).Body1;
j = TraRev(jointCount).Body2;
% Bodies Position Vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Bodies Euler Parameters
pi = Impose_Column(Bodies(i).p);
pj = Impose_Column(Bodies(j).p);
%Joint Location relative to each bodie frame
spi = Impose_Column(TraRev(jointCount).spi);
spj = Impose_Column(TraRev(jointCount).spj);
%Definition of the vectors in body coord system
si = Impose_Column(TraRev(jointCount).si);
sj = Impose_Column(TraRev(jointCount).sj);
%Rotation Matrix
Ai = Bodies(i).A;
Aj = Bodies(j).A;
%Vector sj in the global frame
sig = Ai*si;
sjg = Aj*sj;
%Skew Matrix body
ssi = SkewMatrix4(si);
ssj = SkewMatrix4(sj);
%SkewMatrix Vector si and sj for P
sspi = SkewMatrix4(spi);
sspj = SkewMatrix4(spj);
% Euler Parameters Aux Identities
Gi = Bodies(i).G;
Gj = Bodies(j).G;
%Global Vector for P
spig = Ai*spi;
spjg = Aj*spj;

%Vector between P's of the Bodies
d = rj + spjg - ri - spig;
%Perpendicular Vector do si
[~,ud] = unitvector(d);
qig = cross(sig,ud);
qi = Ai'*qig;
sqi = SkewMatrix4(qi);

% 2 non-colinear vector that are perpendicular to the vector si
[qj,~] = PerpendicularVectors(sj);
%Perpendicular vectors in the global frame
%Body j
qjg = Aj*qj;
%SkewMatrix of the perpendicular vectors body frame
%Body j
sqj = SkewMatrix4(qj);

%% Joint Formulation - Kinematic Problem
% Form the position constraint equations
if(Flags.Position == 1)
    fun(funCount,1)   = qig'*d;
    fun(funCount+1,1) = sig'*d;
    fun(funCount+2,1) = qig'*qjg;
    fun(funCount+3,1) = sig'*sjg;
end

if (Flags.Jacobian == 1) && (Flags.Dynamic == 0)
    Bi = 2*(Gi*sspi + spi*pi');
    Bj = 2*(Gj*sspj + spj*pj');
    Ciq = 2*(Gi*sqi + qi*pi');
    Cis = 2*(Gi*ssi + si*pi');
    Cjs = 2*(Gj*ssj + sj*pj');
    Cjq = 2*(Gj*sqj + qj*pj');
    %Body i
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [-qig',-qig'*Bi+d'*Ciq];
    Jacobian(funCount+1,i1:i2) = [-sig',-sig'*Bi+d'*Cis];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,qjg'*Ciq];
    Jacobian(funCount+3,i1:i2) = [0,0,0,sjg'*Cis];
    %Body j
    i1 = 7*(j-1)+1;
    i2 = i1+6;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [qig',qig'*Bj];
    Jacobian(funCount+1,i1:i2) = [sig',sig'*Bj];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,qig'*Cjq];
    Jacobian(funCount+3,i1:i2) = [0,0,0,sig'*Cjs];
end

% Form the r.h.s velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+3,1) = 0;
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
    qid = Ai*SkewMatrix3(wli)*qig;
    sid = Ai*SkewMatrix3(wli)*sig;
    sjd = Aj*SkewMatrix3(wlj)*sjg;
    qjd = Aj*SkewMatrix3(wlj)*qjg;
    %For the derivative of d we have to use the eq defined above in the
    %code d = rj + Aj*spj - ri - Ai*spi;
    rid = Bodies(i).rd;
    rjd = Bodies(j).rd;
    dd =  rjd + Aj*SkewMatrix3(wlj)*spj - rid - Ai*SkewMatrix3(wli)*spi;
    %Spid and Spjd for the 2nd perpendiculars
    spid = SkewMatrix3(wli)*spi;
    spjd = SkewMatrix3(wlj)*spj;
    
    gamma(funCount,1) = qig'*(-2*Gdj*Ldj'*spjd + 2*Gdi*Ldi'*spid) + d'*(-2*Gdi*Ldi'*qi) - 2*dd'*qid; 
    gamma(funCount+1,1) = sig'*(-2*Gdj*Ldj'*spjd + 2*Gdi*Ldi'*spid) + d'*(-2*Gdi*Ldi'*si) - 2*dd'*sid;
    gamma(funCount+2,1) = qig'*(-2*Gdj*Ldj'*qj) + qjg'*(-2*Gdi*Ldi'*qi) - 2*qid'*qjd; 
    gamma(funCount+3,1) = sig'*(-2*Gdj*Ldj'*sj) + sjg'*(-2*Gdi*Ldi'*si) - 2*sid'*sjd;    

end

%% Joint Formulation - Dynamic Problem

% Form the Jacobian Matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 1)
    % 2x n2,1 and 2x n1,1 (Table 11.1)
    %Skew Matrix 3x3
    skewqi = SkewMatrix3(qig);
    skewsi = SkewMatrix3(sig);
    skewsj = SkewMatrix3(sjg);
    skewqj = SkewMatrix3(qjg);
    %Body i
    i1 = 6*(i-1)+1;
    i2  = i1+5;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [-qig',-(d+spig)'*skewqi*Ai];
    Jacobian(funCount+1,i1:i2) = [-sig',-(d+spig)'*skewsi*Ai];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,-qjg'*skewqi*Ai];
    Jacobian(funCount+3,i1:i2) = [0,0,0,-sjg'*skewsi*Ai];
    %Body j
    i1 = 6*(j-1)+1;
    i2 = i1+5;
    %Perp Type 2
    Jacobian(funCount,i1:i2) = [qig',-qig'*skewsj*Aj];
    Jacobian(funCount+1,i1:i2) = [sig',-sig'*skewsj*Aj];
    %Perp Type 1
    Jacobian(funCount+2,i1:i2) = [0,0,0,-qig'*skewqj*Aj];
    Jacobian(funCount+3,i1:i2) = [0,0,0,-sig'*skewsj*Aj];
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
    spid = swi*spig;
    spjd = swj*spjg;
    %Derivatives of qi,ti and sj in the global frame
    qid = swi*qig;
    sid = swi*sig;
    sjd = swj*sjg;
    qjd = swj*qjg;
    
    % d vector derivative -> A was taken out because the velocities given
    % are in the absolute frame
    dd = rdj + spjd - rdi - spid;
    dd = Impose_Column(dd);
    
    gamma(funCount,1) = -2*(dd)'*qid - d'*swi*qid + qig'*(swi*spid - swj*spjd);
    gamma(funCount+1,1) = -2*(dd)'*sid - d'*swi*sid + sig'*(swi*spid - swj*spjd);
    gamma(funCount+2,1) = -2*(qid)'*qjd + qid'*swi*qjg + qjd'*swj*qig;
    gamma(funCount+3,1) = -2*(sid)'*sjd + sid'*swi*sjg + sjd'*swj*sig;
end

%% Update the line counter
funCount = funCount+4;

end

