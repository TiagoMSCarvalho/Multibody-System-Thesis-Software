function [fun,Jacobian,niu,gamma,funCount] = Joint_CompSphRev(fun,Jacobian,niu,gamma,funCount,jointCount,Bodies,CompSphRev,Flags)
%% Initial variable definitions
% Bodies numbers
i = CompSphRev(jointCount).Body1;
j = CompSphRev(jointCount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Bodies Euler Parameters
pi = Impose_Column(Bodies(i).p);
pj = Impose_Column(Bodies(j).p);
% Joint location relative to each Bodies coordinate system
spi = Impose_Column(CompSphRev(jointCount).spi);
spj = Impose_Column(CompSphRev(jointCount).spj);
% Rev Joint Axis
sj = Impose_Column(Revolute(jointCount).sj);
%Skew Matrix 4x4
sspi = SkewMatrix4(spi);
sspj = SkewMatrix4(spj);
% Rotation matrix for each Bodies
Ai = Bodies(i).A;
Aj = Bodies(j).A;
%Vector sj in the global frame
sjg = Aj*sj;
% Euler Parameters Aux Identities
Gi = Bodies(i).G;
Gj = Bodies(j).G;
%Joint location in the global/absolute coordinate system
spig = Ai*spi;
spjg = Aj*spj;
%Joint Link length
len = CompSphRev(joinCount).length;
%Definition of the d vector
d = rj + spjg - ri - spig;
d = Impose_Column(d);


%% Joint Formulation - Kinematic Problem
% Position constraint equations
if(Flags.Position == 1)
    fun(funCount,1) = d'*d - len^2;
    fun(funCount+1,1) = d'*sjg;
end

% Jacobian Matrix
% eye command that returns identity matrix
if (Flags.Jacobian == 1) && (Flags.Dynamic == 0)
    %Ci Cj aux calc tab 7.1 Nikra  (pg291)
    %Constrain defined relative to the point P central to the sph joint
    Bi = 2*(Gi*sspi + spi*pi');
    Bj = 2*(Gj*sspj + spj*pj');
    Ci = 2*(Gi*sspi + spi*pi');
    %Body i
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount,i1:i2)=[-2*d',-2*d'*Bi];
    Jacobian(funCount+1,i1:i2)=[-sjg',-sjg'*Bi+d'*Ci];
    %Body j
    i1 = 7*(j-1)+1;
    i2  = i1+6;
    Jacobian(funCount,i1:i2)=[2*d',2*d'*Bj];
    Jacobian(funCount+1,i1:i2)=[sjg',sjg'*Bj];
end

% Right-hand-side velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+1,1)=0;
end

% Right-hand-side acceleration equations
if(Flags.Acceleration == 1)       
    
    %Taking the G and L derivatives out
    Gdi = Bodies(i).Gd;
    Gdj = Bodies(j).Gd;
    Ldi = Bodies(i).Ld;
    Ldj = Bodies(j).Ld;
    %Calculating hi and hj
    hi = -2*Gdi*Ldi'*spi;
    hj = -2*Gdj*Ldj'*spj;
    %Extract the angular velocity vectors from the Bodies Struct
    wli = Bodies(i).wl;
    wlj = Bodies(j).wl;
    %Sj Derivatives
    sjd = Aj*SkewMatrix3(wlj)*sjg;
    %Derivative of d
    rid = Bodies(i).rd;  
    rjd = Bodies(j).rd;
    dd = rjd + Aj*SkewMatrix3(wlj)*spj - rid - Ai*SkewMatrix3(wli)*spi;
    dd = Impose_Column(dd);
    
    gamma(funCount,1) = 2*d'*(hi-hj) - 2*(dd')*dd;
    gamma(funCount+1,1) = sjg'*(hj-hi) + d'*hi - 2*sjd'*dd;
end
%% Joint Formulation - Dynamic Problem
if (Flags.Jacobian == 1) && (Flags.Dynamic == 1)
    skewspi = SkewMatrix3(spig);
    skewspj = SkewMatrix3(spjg);
    %Body i
    i1 = 6*(i-1)+1;
    i2  = i1+5;
    Jacobian(funCount,i1:i2)=[-2*d',2*d'*skewspi*Ai];
    Jacobian(funCount+1,i1:i2)=[-sjg',-(d+spjg)'*skewspj*Ai];
    %Body j
    i1 = 6*(j-1)+1;
    i2  = i1+5;
    Jacobian(funCount,i1:i2)=[2*d',-2*d'*skewspj*Aj];
    Jacobian(funCount+1,i1:i2)=[sjg',-sjg'*skewspj*Aj];
end

if (Flags.AccelDyn == 1)
    %Body i, dynamic pre processing
    rdi = Bodies(i).rd;
    wi = Bodies(i).w;
    %Body j, dynamic pre processing
    rdj = Bodies(j).rd;
    wj = Bodies(j).w;
    %Sj Derivatives
    sjd = Aj*SkewMatrix3(wj)*sjg;
    %Derivatives of the sp's
    spid = Ai*SkewMatrix3(wi)*spi;
    spjd = Aj*SkewMatrix3(wj)*spj;
    % d vector derivative -> A was taken out because the velocities given
    % are in the absolute frame
    dd = rdj + spjd - rdi - spid;
    dd = Impose_Column(dd);
    gamma(funCount,1) = -2*(dd)'*dd + 2*d'*(SkewMatrix3(wi)*spid - SkewMatrix3(wj)*spjd);
    gamma(funCount+1,1) = -2*(dd)'*sjd + d'*(SkewMatrix3(wj))*sjd + sjg'*(SkewMatrix3(wj)*spjd - SkewMatrix3(wi)*spid);
end
   
%% Update the function counter
funCount = funCount+1;
end

