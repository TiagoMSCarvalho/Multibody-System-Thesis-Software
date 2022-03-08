function [Bodies] = AccelerationPreDataTreat(~,NBodies,Bodies)
%Treats the data retrieved from the velocity analysis in order to establish
%the Ld and Gd matrixes.
for i=1:NBodies
    %%Calculation of the Gd and Ld used for the gamma formulation
    pd = Impose_Column(Bodies(i).pd);    
    e0d = pd(1); 
    e1d = pd(2);
    e2d = pd(3);
    e3d = pd(4);
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
    %pd = Bodies(i).pd;
    %%Calc of the Angular Veloctiy following 6.105 Nikravesh equation and
    %%used for the calculation of derivatives.
    G = Bodies(i).G;
    L = Bodies(i).L;
    %Global Ang.Vel vector:
    Bodies(i).wg = 2*G*pd;
    %Local Ang.Vel vector:
    Bodies(i).wl = 2*L*pd;
    %This Angular velocity vector will be used to calculate the derivatives
    %of the vector for the 7.2 Table, following the 6.101 formula Nikra Pg
    %174
                
end
end

