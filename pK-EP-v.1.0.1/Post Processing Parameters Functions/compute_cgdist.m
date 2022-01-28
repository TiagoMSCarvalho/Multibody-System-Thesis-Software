function [CG2RC]=compute_cgdist(RC,Points,i)

CG=[Points{69,1}(i) Points{69,2}(i) Points{69,3}(i)];



CG2RC=sqrt((RC(2)-CG(2))^2+(RC(3)-CG(3))^2);