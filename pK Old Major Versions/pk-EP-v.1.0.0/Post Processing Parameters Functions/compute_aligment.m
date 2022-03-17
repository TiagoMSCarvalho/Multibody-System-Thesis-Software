function [Camber,Toe]=compute_aligment(Points,i,FlagWheel)

cambervector=[0,0,1];
toevector=[1,0,0];

FlagWheel=upper(FlagWheel);

if FlagWheel=='FL'
    WheelFront=9;
    WheelRear=10;
    WheelDown=11;
    WheelTop=12;
    flagcamber=1;
    flagtoe=1;
elseif FlagWheel=='FR'
    WheelFront=21;
    WheelRear=22;
    WheelDown=23;
    WheelTop=24;
    flagcamber=-1;
    flagtoe=-1;
elseif FlagWheel=='RL'
    WheelFront=33;
    WheelRear=34;
    WheelDown=35;
    WheelTop=36;
    flagcamber=1;
    flagtoe=1;
elseif FlagWheel=='RR'
    WheelFront=45;
    WheelRear=46;
    WheelDown=47;
    WheelTop=48;
    flagcamber=-1;
    flagtoe=-1;
end

for j=1:3
FrontRearVector(j)=Points{WheelFront,j}(i)-Points{WheelRear,j}(i);
TopBottomVector(j)=Points{WheelTop,j}(i)-Points{WheelDown,j}(i);
end

normalvector=cross(FrontRearVector,TopBottomVector);%wheel plane

normalvector=normalvector/norm(normalvector);

Camber=flagcamber*asind(dot(normalvector,cambervector));
Toe=flagtoe*asind(dot(normalvector,toevector));