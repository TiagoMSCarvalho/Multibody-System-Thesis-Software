function [IC_L,IC_R,RC]=compute_centres(Points,i,AxleFlag)


% Upper,Left
% 1 2 3
% Lower,Left
% 4 5 6
% Upper,Right
% 13 14 15
% Lower, Left
% 16 17 18
% Wheel Down, Left
% 11
% Wheel Down, Right
% 23


if AxleFlag=='F'
    idx=[1 2 3 4 5 6 13 14 15 16 17 18 11 23];%points for the front axle
    xAxle=0;
    xAxle=Points{idx(length(idx)),1}(i);
elseif AxleFlag=='R'
    idx=[25 26 27 28 29 30 37 38 39 40 41 42 35 47];%corresponding points for the rear axle
    xAxle=Points{idx(length(idx)),1}(i);
end


%set vectors for each plane to define (UL -> upper A-arm Left)
%% VECTORS

for j=1:3
    
%upper A-arm Left
UL1(j)=Points{idx(3),j}(i)-Points{idx(1),j}(i);
UL2(j)=Points{idx(3),j}(i)-Points{idx(2),j}(i);

%lower A-arm Left
LL1(j)=Points{idx(6),j}(i)-Points{idx(4),j}(i);
LL2(j)=Points{idx(6),j}(i)-Points{idx(5),j}(i);

%upper A-arm Right
UR1(j)=Points{idx(9),j}(i)-Points{idx(7),j}(i);
UR2(j)=Points{idx(9),j}(i)-Points{idx(8),j}(i);

%lower
LR1(j)=Points{idx(12),j}(i)-Points{idx(10),j}(i);
LR2(j)=Points{idx(12),j}(i)-Points{idx(11),j}(i);
end


%% PLANES

UL=cross(UL1,UL2);
UL=UL/norm(UL);
LL=cross(LL1,LL2);
LL=LL/norm(LL);
UR=cross(UR1,UR2);
UR=UR/norm(UR);
LR=cross(LR1,LR2);
LR=LR/norm(LR);

%upright points to find d value for plane in ax+by+cz=d
dUL=(UL(1)*Points{idx(3),1}(i)+UL(2)*Points{idx(3),2}(i)+UL(3)*Points{idx(3),3}(i))/norm(UL);
dLL=(LL(1)*Points{idx(6),1}(i)+LL(2)*Points{idx(6),2}(i)+LL(3)*Points{idx(6),3}(i))/norm(LL);

dUR=(UR(1)*Points{idx(9),1}(i)+UR(2)*Points{idx(9),2}(i)+UR(3)*Points{idx(9),3}(i))/norm(UR);
dLR=(LR(1)*Points{idx(12),1}(i)+LR(2)*Points{idx(12),2}(i)+LR(3)*Points{idx(12),3}(i))/norm(LR);

%% Left Side

%vetorial equation of the plane intersection: r=q+t*n
n=UL*LL'; %helper for left side
qL=(UL*(dUL-dLL*n)+LL*(dLL-dUL*n))/(1-n^2);
nL=cross(UL,LL);%directing vector of the intersection of the planes

%% Right Side

%vetorial equation of the plane intersection: r=q+t*n
n=UR*LR'; %helper for left side
qR=(UR*(dUR-dLR*n)+LR*(dLR-dUR*n))/(1-n^2);
nR=cross(UR,LR);%directing vector of the intersection of the planes

%% Solve the equations

tL=(-qL(1)+xAxle)/nL(1);
tR=(-qR(1)+xAxle)/nR(1);

IC_L=qL+tL*nL;
IC_R=qR-tL*nR;

%% Roll Centre calculation

vL=IC_L-[Points{idx(13),1}(i) Points{idx(13),2}(i) Points{idx(13),3}(i)];
vR=IC_R-[Points{idx(14),1}(i) Points{idx(14),2}(i) Points{idx(14),3}(i)];

%solving the vectorial equation for both straights (connecting the IC's and
%respective wheels)
%[x,y,z]=p(x,y,z)+k*v(x,y,z)

vector=[Points{idx(13),2}(i)-Points{idx(14),2}(i) Points{idx(13),3}(i)-Points{idx(14),3}(i)];
matrix=[-vL(2) -vL(3);vR(2) vR(3)];

k=vector*inv(matrix);

RC=[Points{idx(13),1}(i) Points{idx(13),2}(i) Points{idx(13),3}(i)]+k(1)*vL;