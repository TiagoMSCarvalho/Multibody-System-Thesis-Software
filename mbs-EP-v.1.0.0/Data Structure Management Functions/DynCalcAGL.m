function Bodies = DynCalcAGL(q0, NBodies, Bodies)
% Update the position, orientation and transformation matrices
for i=1:NBodies
    l1 = 7*(i-1)+1;
    r = Impose_Column(q0(l1:l1+2));
    p = Impose_Column(q0(l1+3:l1+6));
    
    %Above is Corrected to retrieve the Euler Parameters
    
    e0 = p(1);
    e1 = p(2);
    e2 = p(3);
    e3 = p(4);
    
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

