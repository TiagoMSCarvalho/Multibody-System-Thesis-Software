function [Joints,Heave,Roll,Pitch]=ImposeDrivers(Bodies,Joints)



Roll=0;
Heave=0;
Pitch=0;
steeringratio=75;
trackfront=Bodies(2).r(2)-Bodies(7).r(2);
trackrear=Bodies(11).r(2)-Bodies(15).r(2);
wheelbase=Bodies(2).r(1)-Bodies(11).r(1);
%insert heave in mm, roll in deg, steering in deg
%inputs ao utilizador para definir nr de drivers
FlagHeave=upper(input('Do you want Heave inputs? [y/n]\n','s'));

FlagRoll=upper(input('Do you want Roll inputs? [y/n]\n','s'));

FlagPitch=upper(input('Do you want Pitch inputs? [y/n]\n','s'));

FlagSteering=upper(input('Do you want Steering inputs? [y/n]\n','s'));

%inputs necessários para os motions
if FlagHeave=='Y'
    Heave=input('Input Heave value in mm\n');
end
if FlagRoll=='Y'
    Roll=input('Input Roll value in degrees\n');
    Roll=deg2rad(Roll);
end

if FlagPitch=='Y'
    Pitch=input('Input Pitch value in degrees\n');
    Pitch=deg2rad(Pitch);
end
%driver nr1 é sempre steering, haja só 1 ou 5
%defenir um valor standard, caso não se queira driver nenhum
Steering=0;
if FlagSteering=='Y'
    Steering=input('Input Steering value in degrees of steering wheel rotation\n');
end
rackdisplacement=Steering/360*steeringratio;

%driver de steering - sempre definido porque o programa precisa de 1 driver
Joints.Driver(1).Body=6;
Joints.Driver(1).pos0=rackdisplacement;
Joints.Driver(1).v0=0;
Joints.Driver(1).a0=0;
Joints.Driver(1).direction=2;

%se não houver motions apenas define o steering, a 0, para o programa
%correr. Se houver alguma motion pedida, há 5 drivers, 1 rack + 4 rodas.
if FlagHeave=='Y'||FlagRoll=='Y'||FlagPitch=='Y'
    %ordem de rodas:[fl fr rl rr]
    disp_roll_f=sin(Roll)*trackfront/2;
    disp_roll_r=sin(Roll)*trackrear/2;
    disp_pitch=sin(Roll)*wheelbase/2;
    roll_matrix=[[1 -1]*disp_roll_f [1 -1]*disp_roll_r];
    pitch_matrix=[1 1 -1 -1]*disp_pitch;
    Heave=[1 1 1 1]*Heave;
    
    
    %roda fl
    Joints.Driver(2).Body=2;
    Joints.Driver(2).pos0=roll_matrix(1)+Heave(1)+pitch_matrix(1);
    Joints.Driver(2).v0=0;
    Joints.Driver(2).a0=0;
    Joints.Driver(2).direction=3;
    %roda fr
    Joints.Driver(3).Body=7;
    Joints.Driver(3).pos0=roll_matrix(2)+Heave(2)+pitch_matrix(2);
    Joints.Driver(3).v0=0;
    Joints.Driver(3).a0=0;
    Joints.Driver(3).direction=3;
    %roda rl
    Joints.Driver(4).Body=11;
    Joints.Driver(4).pos0=roll_matrix(3)+Heave(3)+pitch_matrix(3);
    Joints.Driver(4).v0=0;
    Joints.Driver(4).a0=0;
    Joints.Driver(4).direction=3;
    %roda rr
    Joints.Driver(3).Body=15;
    Joints.Driver(3).pos0=roll_matrix(4)+Heave(4)+pitch_matrix(4);
    Joints.Driver(3).v0=0;
    Joints.Driver(3).a0=0;
    Joints.Driver(3).direction=3;
    
    
end

%proximas coisas a acrescentar: comentarios, ciclo para determinar
%automaticamente os bodies nos quais se põe drivers
