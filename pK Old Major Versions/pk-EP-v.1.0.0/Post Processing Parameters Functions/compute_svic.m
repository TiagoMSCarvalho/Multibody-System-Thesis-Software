function [SVIC]=compute_svic(Points,idx1,flag)

flag=upper(flag);

if flag=='F'
    idx= [1 2 4 5];
elseif flag=='R'
    idx= [26 25 29 28];
end
%y não tem interesse
p1=[Points{idx(1),1}(idx1) Points{idx(1),3}(idx1)];
p2=[Points{idx(2),1}(idx1) Points{idx(2),3}(idx1)];
p3=[Points{idx(3),1}(idx1) Points{idx(3),3}(idx1)];
p4=[Points{idx(4),1}(idx1) Points{idx(4),3}(idx1)];

vec1=p2-p1; %vector for the vectorial equation of the first line
vec1=vec1/norm(vec1);
vec2=p4-p3; %vector for the vectorial equation of the second line
vec2=vec2/norm(vec2);

%[x z]=p[x z]+k[v1 v2]

matrix=[vec1;vec2];
vector=[p3(1)-p1(1) p3(2)-p1(2)];
k=vector*inv(matrix);

SVIC=p1+k(1)*vec1;
