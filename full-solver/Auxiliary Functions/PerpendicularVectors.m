function [qi,ti] = PerpendicularVectors(vecsi)
    %Calculates two orthogonal vectors to the vector si, to avoid the use
    %of the parallel constrain due to the possibility of the cross product
    %generating a row of 0 in the Jac Matrix and thus a singularity
    
    qi = [0;0;0];
    ti = [0;0;0];
    vecsi = Impose_Column(vecsi);
    
    if vecsi(1) >0 || vecsi(1) < 0
        x = (+vecsi(2)+vecsi(3))/vecsi(1);
        qi = [x; -1; -1]; %Construct and Normalize
        ti = SkewMatrix3(qi)*vecsi; %Cross-Product
    elseif vecsi(2) >0 || vecsi(2) <0
        y = (+vecsi(1)+vecsi(3))/vecsi(2);
        qi = [-1; y; -1];
        ti = SkewMatrix3(qi)*vecsi;
    elseif vecsi(3) >0 || vecsi(3) <0
        z = (+vecsi(2)+vecsi(1))/vecsi(3);
        qi = [-1; -1; z];
        ti = SkewMatrix3(qi)*vecsi;
    end

end

