function [Ackerman,Ackerman_percent,InstantTurningRadius]=compute_ackerman(Toe_FL,Toe_FR,WheelBase_L,WheelBase_R,Joints,i,timestep,Track_F,Points)

pos_rack=Joints.Driver(1).pos0;
vel_rack=Joints.Driver(1).v0;
ac_rack=Joints.Driver(1).a0;
t=(i-1)*timestep;

pos=pos_rack+vel_rack*t+1/2*ac_rack*(t^2);

if pos>0
    deltainside=abs(deg2rad(Toe_FL));
    deltaoutside=abs(deg2rad(Toe_FR));
else
    deltainside=abs(deg2rad(Toe_FL));
    deltaoutside=abs(deg2rad(Toe_FR));
end



WheelBase=(WheelBase_L+WheelBase_R)/(2*1000);
Ackerman=atan(WheelBase/(WheelBase/tan(deltaoutside)-Track_F/1000));
Ackerman_percent=deltainside/Ackerman*100;

CG=[Points{69,1}(i) Points{69,2}(i) Points{69,3}(i)];
a2=abs(WheelBase-CG(1))/1000;

delta=acot(1/2*(cot(deltainside)+cot(deltaoutside)));
InstantTurningRadius=sqrt(a2^2+(WheelBase^2)*(cot(delta)^2));
