function qd = CreateVelocityVector(NBodies,Bodies)
%Allocates and creates a velocity vector for the Baumgarte's algorithm
    for i=1:NBodies
        l1 = 6*(i-1)+1;
        qd(l1:l1+2,1) = Bodies(i).rd;
        qd(l1+3:l1+5,1) = Bodies(i).w;
    end
end

