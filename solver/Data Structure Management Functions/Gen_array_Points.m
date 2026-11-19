function [Points_arrays] = Gen_array_Points(Points, idx)

for i=1:(length(Points)-1)
    for j=1:3
        Points_arrays(i,j) = Points{i,j}(idx);
    end
end

end

