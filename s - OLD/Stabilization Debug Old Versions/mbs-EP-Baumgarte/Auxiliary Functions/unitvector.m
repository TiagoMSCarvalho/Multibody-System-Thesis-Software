function [magnitude,uvector] = unitvector(vector)
%Calculates the magnitude and direction of a pre given vector
    magnitude = norm(vector);
    
    uvector = vector/magnitude;
    uvector = Impose_Column(uvector);
    if magnitude == 0
        uvector = [0;0;0];
    end
    
end

