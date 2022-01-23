function [parameters]=sus_param_calculator(Points,PointsStatic,Joints,TimeStep)

%INPUTS: -Points: cell array with the location of each point along the
%various iterations
%        -PointsStatic: cell array with the location of each point, before
%        any motions
%OUTPUTS:-Parameters:struct with all the relevant parameters, along all
%points in time.

%Scope: Compute the needed values when considering the points given by the
%3D multibody simulation

% parameters ={'camber FL', 'camber FR', 'toe FL', 'toe FR', 'IC left', ...
%     'IC right', 'Roll Centre', 'kingpin inclination FL', 'kingpin inclination FR', ...
%     'caster FL', 'caster FR'};

%Note:variable/fucntion names indicate their purpose. Toe_FL represents the
%toe values for the front left wheel, per example.



%compute the values wanted
for i=1:length(Points{1,1})
   [Camber_FL(i),Toe_FL(i)]=compute_aligment(Points,i,'FL');
   [Caster_FL(i),Kingpin_FL(i),Scrubradius_FL(i)]=compute_caster(Points,i,'FL');
   [Camber_FR(i),Toe_FR(i)]=compute_aligment(Points,i,'FR');
   [Caster_FR(i),Kingpin_FR(i),Scrubradius_FR(i)]=compute_caster(Points,i,'FR');
   [IC_FL{i},IC_FR{i},RC_F{i}]=compute_centres(Points,i,'F');
   [Camber_RL(i),Toe_RL(i)]=compute_aligment(Points,i,'RL');
   [Caster_RL(i),Kingpin_RL(i),Scrubradius_RL(i)]=compute_caster(Points,i,'RL');
   [Camber_RR(i),Toe_RR(i)]=compute_aligment(Points,i,'RR');
   [Caster_RR(i),Kingpin_RR(i),Scrubradius_RR(i)]=compute_caster(Points,i,'RR');
   [IC_RL{i},IC_RR{i},RC_R{i}]=compute_centres(Points,i,'R');
   [Track_F(i),Track_R(i),WheelBase_L(i),WheelBase_R(i)]=compute_wb(Points,i);
   [Coilover_FL{i}]=compute_coilovers(Points,PointsStatic,Joints,i,'FL',TimeStep);
   [Coilover_FR{i}]=compute_coilovers(Points,PointsStatic,Joints,i,'FR',TimeStep);
   [Coilover_RL{i}]=compute_coilovers(Points,PointsStatic,Joints,i,'RL',TimeStep);
   [Coilover_RR{i}]=compute_coilovers(Points,PointsStatic,Joints,i,'RR',TimeStep);
   [Antis{i},SVIC_F{i},SVIC_R{i}]=compute_antis(Points,i);
   [Ackerman(i),Ackerman_percent(i),InstantTurningRadius(i)]=compute_ackerman(Toe_FL(i),Toe_FR(i),WheelBase_L(i),WheelBase_R(i),Joints,i,TimeStep,Track_F(i),Points);
   [CG2RC_F(i)]=compute_cgdist(RC_F{i},Points,i);
   [CG2RC_R(i)]=compute_cgdist(RC_R{i},Points,i);
end


%write the values to the structure
parameters.camberFL=Camber_FL;
parameters.camberFR=Camber_FR;
parameters.toeFL=Toe_FL;
parameters.toeFR=Toe_FR;
parameters.CasterFL=Caster_FL;
parameters.CasterFR=Caster_FR;
parameters.KingPinInclinationFL=Kingpin_FL;
parameters.KingPinInclinationFR=Kingpin_FR;
parameters.Scrubradius_FL=Scrubradius_FL;
parameters.Scrubradius_FR=Scrubradius_FR;
parameters.IC_FL=IC_FL;
parameters.IC_FR=IC_FR;
parameters.RollCentre_F=RC_F;

parameters.camberRL=Camber_RL;
parameters.camberRR=Camber_RR;
parameters.toeRL=Toe_RL;
parameters.toeRR=Toe_RR;
parameters.CasterRL=Caster_RL;
parameters.CasterRR=Caster_RR;
parameters.KingPinInclinationRL=Kingpin_RL;
parameters.KingPinInclinationRR=Kingpin_RR;
parameters.Scrubradius_RL=Scrubradius_RL;
parameters.Scrubradius_RR=Scrubradius_RR;
parameters.IC_RL=IC_RL;
parameters.IC_RR=IC_RR;
parameters.RollCentre_R=RC_R;

parameters.TrackFront=Track_F;
parameters.TrackRear=Track_R;
parameters.WheelBaseLeft=WheelBase_L;
parameters.WheelBaseRight=WheelBase_R;

parameters.CoiloverFL=Coilover_FL;
parameters.CoiloverFR=Coilover_FR;
parameters.CoiloverRL=Coilover_RL;
parameters.CoiloverRR=Coilover_RR;

parameters.Antis=Antis;
parameters.SVIC_F=SVIC_F;
parameters.SVIC_R=SVIC_R;
parameters.Ackerman=Ackerman;
parameters.Ackerman_percent=Ackerman_percent;
parameters.InstantTurningRadius=InstantTurningRadius;

parameters.CGtoRCdist_F=CG2RC_F;
parameters.CGtoRCdist_R=CG2RC_R;