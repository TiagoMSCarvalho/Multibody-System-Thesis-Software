function [Coilover]=compute_coilovers(Points,PointsStatic,Joints,i,flag,timestep)


% flag=upper(flag);
% if flag == 'FL'
%     idx = [52 53];
%     jointidx=2;
% elseif flag == 'FR'
%     idx = [57 58];
%     jointidx=3;
% elseif flag == 'RL'
%     idx = [62 63];
%     jointidx=4;
% elseif flag == 'RR'
%     idx = [67 68];
%     jointidx=5;
% end

point1= [Points{idx(1),1}(i), Points{idx(1),2}(i), Points{idx(1),3}(i)];
point2= [Points{idx(2),1}(i), Points{idx(2),1}(i), Points{idx(2),3}(i)];

point1static= [PointsStatic{idx(1),1}, PointsStatic{idx(1),2}, PointsStatic{idx(1),3}];
point2static= [PointsStatic{idx(2),1}, PointsStatic{idx(2),1}, PointsStatic{idx(2),3}];

driverdisp=Joints.Driver(jointidx).pos0+Joints.Driver(jointidx).v0*(i-1)*timestep+...
    Joints.Driver(jointidx).a0*(((i-1)*timestep)^2);

Coilover.length=norm(point1-point2);
Coilover.lengthstatic=norm(point1static-point2static);
Coilover.extension=Coilover.length-Coilover.lengthstatic;
Coilover.instalationratio=Coilover.extension/(driverdisp);
end


    