% This function makes the input vector be a column array
function vector = Impose_Column(vector)
%Transforms the Vector in Column Vector, iscolumn returns a Logical value (
%1 or 0) if it is not a column it makes its transpose vector' command.
if ~iscolumn(vector)
    vector = vector';
end



