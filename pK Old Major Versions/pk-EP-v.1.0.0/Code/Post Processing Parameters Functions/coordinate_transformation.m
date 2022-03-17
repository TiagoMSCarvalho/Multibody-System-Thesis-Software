function [Points]=coordinate_transformation(Points,Motions)

    %Function Scope: Take a cell array with positions, roll and pitch angles
    %and heave displacement to adjust the coordinates to the new referential

    %Inputs:
    %Points: Cell array with the suspension points
    %Roll: roll angle in degrees
    %Pitch: pitch angle in degrees
    %Heave: heave displacement in milimetres

    %Output:
    %Points: Cell array with the suspension points, in the updated referential

    Roll = cell2mat(Motions.Roll);
    Pitch = cell2mat(Motions.Pitch);
    Heave = cell2mat(Motions.Heave);
    PitchDisplacement = cell2mat(Motions.PitchDisplacement);
    
    Roll = deg2rad(Roll);
    Pitch = deg2rad(Pitch);

    % have fun: https://www.continuummechanics.org/rotationmatrix.html
    matrixRoll =   [1 0          0;
                    0 cos(Roll)  -sin(Roll); 
                    0 sin(Roll) cos(Roll)];
    
    matrixPitch =   [cos(Pitch)  0 sin(Pitch); 
                     0           1 0;
                     -sin(Pitch) 0 cos(Pitch)];
                 
    

    %matrixRotation = matrixRoll * matrixPitch;

    for i=1:size(Points,1)
        
        
        elem = Points(i,:);
        
        for j=1:length(elem{1}) % select random coordinate to find length of iterations
            pitchPoint = [0, 0, 0]';%
            coord_iter = Impose_Column([elem{1}(j) elem{2}(j) elem{3}(j)]);
            coord_iter(3)=coord_iter(3)-PitchDisplacement(1,1)/2 + Heave(1,1);
            new_coord = matrixRoll*(matrixPitch*(coord_iter-pitchPoint)+pitchPoint);
            Points{i,1}(j) = new_coord(1);
            Points{i,2}(j) = new_coord(2);
            Points{i,3}(j) = new_coord(3);
            
        end
        
    end
    
end


