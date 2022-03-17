function [Caster,Kingpin,ScrubRadius]=compute_caster(Points,i,FlagSide)

FlagSide=upper(FlagSide);
if FlagSide=='FL'
    Upper=3;
    Lower=Upper+3;
    WheelDown=11;
    FlagKingpin=-1;
    FlagScrubRadius=1;
elseif FlagSide=='FR'
    Upper=15;
    Lower=Upper+3;
    WheelDown=23;
    FlagKingpin=1;
    FlagScrubRadius=-1;
elseif FlagSide=='RL'
    Upper=27;
    Lower=Upper+3;
    WheelDown=35;
    FlagKingpin=-1;
    FlagScrubRadius=1;
elseif FlagSide=='RR'
    Upper=39;
    Lower=Upper+3;
    WheelDown=47;
    FlagKingpin=1;
    FlagScrubRadius=-1;
end

for j=1:3
UpperPoint(j)=Points{Upper,j}(i);
LowerPoint(j)=Points{Lower,j}(i);
ContactPatch(j)=Points{WheelDown,j}(i);
end

DeltaX=UpperPoint(1)-LowerPoint(1);
DeltaY=UpperPoint(2)-LowerPoint(2);
DeltaZ=UpperPoint(3)-LowerPoint(3);


Caster=-atan(DeltaX/DeltaZ);
Kingpin=FlagKingpin*atan(DeltaY/DeltaZ);
ScrubRadius=FlagScrubRadius*(ContactPatch(2)-(LowerPoint(2)+FlagScrubRadius*tan(Kingpin)*LowerPoint(3)));
Caster=rad2deg(Caster);
Kingpin=rad2deg(Kingpin);
