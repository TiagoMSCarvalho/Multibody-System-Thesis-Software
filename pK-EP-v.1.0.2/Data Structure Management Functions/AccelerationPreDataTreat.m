function [Bodies] = AccelerationPreDataTreat(qd,NBodies,Bodies)
%Treats the data retrieved from the velocity analysis in order to establish
%the Ld and Gd matrixes.
for i=1:NBodies
    %%Calculation of the Gd and Ld used for the gamma formulation
    k = (7*(i-1)+1):(7*i); 
    q = qd(k); %isolates the coordinates for the body;
    %first three coordinates are from the translation;
    e0d = q(4); 
    e1d = q(5);
    e2d = q(6);
    e3d = q(7);
    %G and L derivatives assumir que d(G)/dt = [d(e1)/dt...]
    Gd = [-e1d, e0d, -e3d, e2d;
          -e2d, e3d, e0d, -e1d;
          -e3d, -e2d, e1d, e0d];
    Ld = [-e1d, e0d, e3d, -e2d;
          -e2d, -e3d, e0d, e1d;
          -e3d, e2d, -e1d, e0d];
    %Allocation of the derivatives            
    Bodies(i).Gd = Gd;
    Bodies(i).Ld = Ld;
    %For easier debug
    pd = Bodies(i).pd;
    %%Calc of the Angular Veloctiy following 6.105 Nikravesh equation and
    %%used for the calculation of derivatives.
    %Global Ang.Vel vector:
    Bodies(i).wg = 2*Gd*pd;
    %Local Ang.Vel vector:
    Bodies(i).wl = 2*Ld*pd;
    %This Angular velocity vector will be used to calculate the derivatives
    %of the vector for the 7.2 Table, following the 6.101 formula Nikra Pg
    %174
                
end
end

