function [Bodies] = DynIAGdLdcalc(NBodies,Bodies)
%Calculates the Gd and Ld matrices to allow the calculation of the 7
%coordinate rhs of the acceleration vector.
    %FunctioN Similar to AccelerationPredataProcessing but need to have the
    %velocities converted to the EP's.

for i=1:NBodies
    
    %Retrieves the Angular Velocity of the Body
    pd = 0.5*Bodies(i).L'*Bodies(i).w;
    pd = Impose_Column(pd);
    
    %Store pd, to allow the calc of the Euler Parameter Function
    
    Bodies(i).pd = pd;
    
    %%Calculation of the Gd and Ld used for the gamma formulation
    e0d = pd(1); 
    e1d = pd(2);
    e2d = pd(3);
    e3d = pd(4);
    e = [e1d;e2d;e3d];
    
    %G and L derivatives calculation
    Gd = [-e,SkewMatrix3(e) + e0d*eye(3)];
    Ld = [-e,-SkewMatrix3(e) + e0d*eye(3)];
    
    %Allocation of the derivatives to its fiels            
    Bodies(i).Gd = Gd;
    Bodies(i).Ld = Ld;
                
end
end