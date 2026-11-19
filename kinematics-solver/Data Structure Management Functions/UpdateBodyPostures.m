%% Update Postures Function Details
%
% Inputs and Outputs:
%    - Bodies: all the information of the MultiBody System
%    - NBodies: number of bodies in the MultiBody System
%    - q: vector with all the positions and angular orientations of the bodies
%    in the same order as in the Bodies structure. This vector is used to
%    solve the position analysis with fsolve.
%
% Objective of the Function:
%    This function updates the Bodies structure with the obtained
%    positions and orientations from the position analysis. Also, it
%    calculates the rotation matrix each bodies reference (A) and also the
%    derivative of this matrix (B) which is divided in three matrixes (for
%    3D), each matrix to be multiplied by the angle around one of the three
%    axes

function Bodies = UpdateBodyPostures(q, NBodies, Bodies)
% Update the position, orientation and transformation matrices
for i=1:NBodies
    l1 = 7*(i-1)+1;
    Bodies(i).r = Impose_Column(q(l1:l1+2));
    Bodies(i).p = Impose_Column(q(l1+3:l1+6));
    
    %Above is Corrected to retrieve the Euler Parameters
    
    e0 = Bodies(i).p(1);
    e1 = Bodies(i).p(2);
    e2 = Bodies(i).p(3);
    e3 = Bodies(i).p(4);
    
    Bodies(i).A = 2.*[e0^2+e1^2-(1/2), e1*e2-e0*e3, e1*e3+e0*e2;
                    e1*e2+e0*e3, e0^2+e2^2-(1/2), e2*e3-e0*e1;
                    e1*e3-e0*e2, e2*e3+e0*e1, e0^2+e3^2-(1/2)];
                
    %Above Corrected for Euler Parameters
    
    Bodies(i).G = [ -e1,e0,-e3,e2;
                    -e2,e3,e0,-e1;
                    -e3,-e2,e1,e0];
    
    Bodies(i).L = [ -e1,e0,e3,-e2;
                    -e2,-e3,e0,e1;
                    -e3,e2,-e1,e0];
                
end
end