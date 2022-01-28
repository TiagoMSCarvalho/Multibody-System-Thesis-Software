function [skewmatrix4] = SkewMatrix4(vector)
    %Transforms the vector into a 4x4 SkewMatrix for the Pos and Accel
    %Analysis ( Minus Skew Matrix)
    
    skewmatrix4 = [0, -vector';
                   vector, -SkewMatrix3(vector)];
    
end

