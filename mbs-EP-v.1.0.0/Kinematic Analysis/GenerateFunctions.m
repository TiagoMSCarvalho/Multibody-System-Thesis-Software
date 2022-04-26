function fun = GenerateFunctions(q0,Joints,NBodies,Bodies,Flags,t,ang,driverfunctions)

Bodies = UpdateBodyPostures(q0, NBodies, Bodies);
funCount=1;
fun = [];

% For the Ground Constraints
for jointCount=1:Joints.NGround
    [fun,~,~,~,funCount] = Ground_Constraints(fun,[],[],[],funCount,jointCount, Bodies, Joints.Ground,Flags);
end

% For the Spherical Joints
for jointCount=1:Joints.NSpherical
    [fun,~,~,~,funCount] = Joint_Spherical(fun,[],[],[],funCount,jointCount, Bodies, Joints.Spherical,Flags);
end

% For the Composite Spherical Joint (SPH - SPH)
for jointCount=1:Joints.NCompSpherical
    [fun,~,~,~,funCount] = Joint_CompSpherical(fun,[],[],[],funCount,jointCount, Bodies, Joints.CompSpherical,Flags);
end

% For the Universal joints
for jointCount=1:Joints.NUniversal
    [fun,~,~,~,funCount] = Joint_Universal(fun,[],[],[],funCount,jointCount, Bodies, Joints.Universal,Flags); 
end

% Form the Revolute joints
for jointCount=1:Joints.NRevolute
    [fun,~,~,~,funCount] = Joint_Revolute(fun,[],[],[],funCount,jointCount, Bodies, Joints.Revolute,Flags);
end

% For the Cylindrical joints
for jointCount=1:Joints.NCylindrical
    [fun,~,~,~,funCount] = Joint_Cylindrical(fun,[],[],[],funCount,jointCount, Bodies, Joints.Cylindrical,Flags);
end

% For the Translation joints
for jointCount=1:Joints.NTranslation
    [fun,~,~,~,funCount] = Joint_Translation(fun,[],[],[],funCount,jointCount, Bodies, Joints.Translation,Flags);
end

% For the Spherical Revolute joints
for jointCount=1:Joints.NSphRev
    [fun,~,~,~,funCount] = Joint_CompSphRev(fun,[],[],[],funCount,jointCount, Bodies, Joints.CompSphRev,Flags);
end

% For the Translation Revolute Composite joints
for jointCount=1:Joints.NTraRev
    [fun,~,~,~,funCount] = Joint_CompTraRev(fun,[],[],[],funCount,jointCount, Bodies, Joints.CompTraRev,Flags);
end

% For the Simple Constraints
for jointCount=1:Joints.NSimple
    [fun,~,~,~,funCount] = Simple_Constraints(fun,[],[],[],funCount,jointCount, Bodies, Joints.Simple,Flags);
end

%Euler Parameter Constraints
for NBod = 2:NBodies %takes the first body, ground out of the equation
    [fun,~,~,~,funCount] = EulerParameterConstraint(fun,[],[],[],funCount,NBod,Bodies,Flags);
end

% For the Driver Constraints
for jointCount=1:Joints.NDriver
    [fun,~,~,~,funCount] = Driver_Constraints(fun,[],[],[],funCount,jointCount, Bodies, Joints.Driver,Flags,t,driverfunctions);
end
    
    
end