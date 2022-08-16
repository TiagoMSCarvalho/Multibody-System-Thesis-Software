function x = gaussianelim(A,b)
[row,col] = size(A);
n = row;
x = zeros(size(b));
for k = 1:n-1   
  for i = k+1:n
     xMultiplier = A(i,k) / A(k,k);
     for j=k+1:n
        A(i,j) = A(i,j) - xMultiplier * A(k,j);
     end
     b(i, :) = b(i, :) - xMultiplier * b(k, :);
  end
% There is a missing "end" ?!   
    % backsubstitution:
    x(n, :) = b(n, :) / A(n,n);
    for i = n-1:-1:1
        summation = b(i, :);
        for j = i+1:n
            summation = summation - A(i,j) * x(j, :);
        end
        x(i, :) = summation / A(i,i);
    end
end
end

