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
%    - jointCount: current Joint number on the structure Revolute
%    - Bodies: all the information of the MultiBodies System
%    - Revolute: all the information of the Revolute joints present in
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

function [fun,Jacobian,niu,gamma,funCount] = Joint_Revolute(fun,Jacobian,niu,gamma,funCount,jointCount, Bodies, Revolute,Flags)
%% Initial variable definitions
% Bodies numbers
i = Revolute(jointCount).Body1;
j = Revolute(jointCount).Body2;
% Bodies position vectors
ri = Impose_Column(Bodies(i).r);
rj = Impose_Column(Bodies(j).r);
% Bodies Euler Parameters
pi = Impose_Column(Bodies(i).p);
pj = Impose_Column(Bodies(j).p);
% Joint location relative to each Bodies coordinate system
spi = Impose_Column(Revolute(jointCount).spi);
spj = Impose_Column(Revolute(jointCount).spj);
% Definition of the vectors (each one in one of the bodies coordinate
% system) that should be parallel
si = Impose_Column(Revolute(jointCount).si);
sj = Impose_Column(Revolute(jointCount).sj);
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
ssjb = SkewMatrix4(sjg);
%SkewMatrix Vector si and sj for P
sspi = SkewMatrix4(spi);
sspj = SkewMatrix4(spj);
% Euler Parameters Aux Identities
Gi = Bodies(i).G;
Gj = Bodies(j).G;
Li = Bodies(i).L;
Lj = Bodies(j).L;
%Joint location in the global/absolute coordinate system
spig = Ai*spi;
spjg = Aj*spj;


% 2 non-colinear vectors (perpendicular in this case) that are
% perpendicular to the vector si, si is defined in the local fram and thus
% qi and ti will be LOCAL vectors
%qi = [-si(2); si(1); 0];
%ti = cross(si,qi);
[qi,ti]= PerpendicularVectors(si);
%Perpendicular vectors in the global frame
qig = Ai*qi;
tig = Ai*ti;
%SkewMatrix of the perpendicular vectors body frame
sqi = SkewMatrix4(qi);
sti = SkewMatrix4(ti);


%% Joint Formulation
% Position constraint equations
if(Flags.Position == 1)
    fun(funCount:funCount+2,1) = ri + spig - rj - spjg;
    %Use of Perpendicular Vectors to avoid cross product
    fun(funCount+3,1) = qig'*sjg;
    fun(funCount+4,1) = tig'*sjg;
end

% Jacobian Matrix
if (Flags.Jacobian == 1)
    %There is a need to have more than Ci and Cj, this is related to the
    %way that this joint was redefined:
        %First we have the Sph Condition so Ci and Cj are written in relation to the point P
        %Second we have the two perpendicular conditions (type 1) between
        %the vectors  qi, ti and sj to restrict the rotation to one axis
        %this implies:
            %Ciq for vector q, Cit for vector t and a Cj define in relation
            %to sj since this constrain is not written for the point P.
    %Aux C calcs
    Ci = 2*(Gi*sspi + spi*pi');
    Cj = 2*(Gj*sspj + spj*pj');
    Ciq = 2*(Gi*sqi + qi*pi');
    Cit = 2*(Gi*sti + ti*pi');
    Cjs = 2*(Gj*ssj + sj*pj');
    
    %Body i
    i1 = 7*(i-1)+1;
    i2  = i1+6;
    Jacobian(funCount:funCount+2,i1:i2)=[eye(3),Ci];
    Jacobian(funCount+3,i1:i2)=[0,0,0,-sjg'*Ciq]; %Minus Added
    Jacobian(funCount+4,i1:i2)=[0,0,0,-sjg'*Cit]; %Minus Added
    %Body j
    i1 = 7*(j-1)+1;
    i2  = i1+6;
    Jacobian(funCount:funCount+2,i1:i2)=[-eye(3),-Cj];
    Jacobian(funCount+3,i1:i2)=[0,0,0,qig'*Cjs]; %15-02 qi to qig
    Jacobian(funCount+4,i1:i2)=[0,0,0,tig'*Cjs]; %15-02 ti to tig
end

% Right-hand-side velocity equations
if(Flags.Velocity == 1)
    niu(funCount:funCount+4) = 0;
end

% Right-hand-side acceleration equations
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
    %Derivatives of qi,ti and sj in the global frame (eq 6.100/6.101 pg 192)
    qid = SkewMatrix3(wgi)*qig;
    tid = SkewMatrix3(wgi)*tig;
    sjd = SkewMatrix3(wgj)*sjg;
 
    %Following the logic above (Jac) hj will be written in relation to
    %sj and hi will be written in relation to qi and ti
    gamma(funCount:funCount+2) = -2*Gdi*Ldi.'*spi + 2*Gdj*Ldj.'*spj; 
    gamma(funCount+3) = qig'*(-2*Gdj*Ldj'*sj) + sjg'*(-2*Gdj*Ldj'*qi) - 2*qid'*sjd; %15-02 sj to sjg 2nd term.
    gamma(funCount+4) = tig'*(-2*Gdj*Ldj'*sj) + sjg'*(-2*Gdj*Ldj'*ti) - 2*tid'*sjd; %15-02 sj to sjg 2nd term.
end
   
% Update the function counter
funCount = funCount+5;
end