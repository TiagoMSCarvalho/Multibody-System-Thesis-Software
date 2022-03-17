function [Antis,SVIC_F,SVIC_R]=compute_antis(Points,i)

[SVIC_F]=compute_svic(Points,i,'F');
[SVIC_R]=compute_svic(Points,i,'R');

wheelcentreheight=234.9; %tirar o hard code deste valor
frontweight=0.47; %tirar o hardcode
hcg=Points{69,3}(i);

contactpatchf=[Points{11,1}(i) Points{11,2}(i) Points{11,3}(i)];
contactpatchr=[Points{35,1}(i) Points{35,2}(i) Points{35,3}(i)];

wheelcentrer=[Points{35,1}(i) Points{35,2}(i) wheelcentreheight];

wheelbase=Points{11,1}(i)-Points{35,1}(i);

%% antidive
antidive= wheelbase * frontweight * SVIC_F(2)/(hcg * abs(SVIC_F(1)))*100;

%% anti-squat

distx=SVIC_R(1)-contactpatchr(1);
distz=SVIC_R(2)-wheelcentrer(3);
tantheta3=distz/distx;

antisquat=wheelbase * tantheta3/(hcg) *100;

Antis.AntiDive=antidive;
Antis.AntiSquat=antisquat;


