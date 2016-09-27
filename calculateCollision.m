% Written by Patrick Cland
% Shared under the MIT License

function [hasCollided, MTV] = calculateCollision()
    currentFace = [0,0];    %Current face that we are getting the normal from
    currentAxis = [0,0];    %current axis that we are projecting onto
    currentMTV = [0,0];     %current minimum translation vector
    magTrans = intmax('int32');        %this is arbitrary, but this needs to be a large number in the beginning to get rid of false positives
    min1 = 0; min2 = 0; max1 = 0; max2 = 0; %min and max used to detect overlaps
    r1x = 0; r1y = 0; r2x = 0; r2y = 0;     %vertex matrix for each polygon's coordinates


    %retrieve input from user and store in these
    [nFace1, nFace2, vRot1, vRot2, cX1, cX2, cY1, cY2, s1, s2, failure] = processInput();
    if(failure == 1)
        return;
    end
    %create array to store results from dot products, it needs to be as big
    %as many vertices there are on the largest polygon
    dotResult1 = zeros(max([nFace1 nFace2]),1);
    dotResult2 = zeros(max([nFace1 nFace2]),1);

    %create polygons
    r1x = createPolygonX(nFace1, vRot1, cX1, s1);
    r1y = createPolygonY(nFace1, vRot1, cY1, s1);
    r2x = createPolygonX(nFace2, vRot2, cX2, s2);
    r2y = createPolygonY(nFace2, vRot2, cY2, s2);

    %Set up a dynamic bounding region for drawing the polygons on the
    %screen

    boundXmin = min([(cX1-s1) (cX2-s2)]) - 2;
    boundXmax = max([(cX1+s1) (cX2+s2)]) + 2;
    boundYmin = min([(cY1-s1) (cY2-s2)]) - 2;
    boundYmax = max([(cY1+s1) (cY2+s2)]) + 2;

    %Since matlab requires n+1 points to draw an n sided polygon, the last one being
    %the same vertex as the first, do that
    fill(r1x,r1y,'r',r2x,r2y,'b');
    axis([boundXmin boundXmax boundYmin boundYmax]);


    %We must test all the axes from the first polygon against the vertices
    %of the second polygon
    for i = 1:1:nFace1
        if(i == nFace1)
            %The last face is a special case
            currentFace = SubVector( [r1x(nFace1),r1y(nFace1)] , [r1x(1),r1y(1)] );
        else
            currentFace = SubVector( [r1x(i),r1y(i)] , [r1x(i+1),r1y(i+1)] );
        end

        %Get the unit normal vector to the face, which is the axis we
        %project to
        currentAxis = UnitVector(NormalVector(currentFace));
        fprintf('Axis: <%.2f\t%.2f>\n',currentAxis(1),currentAxis(2));

        %do the dot product to get the projection and store it
        for j = 1:1:nFace1
            dotResult1(j) = dot( [r1x(j),r1y(j)] , currentAxis);
        end
        for j = 1:1:nFace2
            dotResult2(j) = dot( [r2x(j),r2y(j)] , currentAxis);
        end

        min1 = min(dotResult1);
        min2 = min(dotResult2);
        max1 = max(dotResult1);
        max2 = max(dotResult2);

        fprintf('Min 1: %.2f\tMax 1: %.2f\tMin 2: %.2f\tMax 2: %.2f\n\n',min1,max1,min2,max2);

        %If there is no overlap of the projections, there couldn't have
        %been a collision
        if(AgtB(min2, max1) == 1)
            fprintf('\nMin 2: %.2f\tMax 1: %.2f\nProjections do not overlap.\nNo collision.\n',min2,max1);
            %hasCollided = 0;
            return;
        end

        %Calculate the magnitude of the minimum translation vector
        %The MTV's magnitude is the value of the overlap of the projections
        %and the direction is the same as the current axis.
        if((max1 - min2) < magTrans)
            magTrans = max1 - min2;
            currentMTV = magTrans*currentAxis;
        end
    end

    %-------------------------------------------------------------------
    %Now we must test all the vertices of the first polygon against the
    %axes from the 2nd polygon
    %-------------------------------------------------------------------
    for i = 1:1:nFace2
        if(i == nFace2)
            %The last face is a special case
            currentFace = SubVector( [r2x(nFace2),r2y(nFace2)] , [r2x(1),r2y(1)] );
        else
            currentFace = SubVector( [r2x(i),r2y(i)] , [r2x(i+1),r2y(i+1)] );
        end

        %Get the unit normal vector to the face, which is the axis we
        %project to
        currentAxis = UnitVector(NormalVector(currentFace));
        fprintf('Axis: <%.2f\t%.2f>\n',currentAxis(1),currentAxis(2));

        for j = 1:1:nFace1
            dotResult1(j) = dot( [r1x(j),r1y(j)] , currentAxis);
        end
        for j = 1:1:nFace2
            dotResult2(j) = dot( [r2x(j),r2y(j)] , currentAxis);
        end

        min1 = min(dotResult1);
        min2 = min(dotResult2);
        max1 = max(dotResult1);
        max2 = max(dotResult2);

        fprintf('Min 1: %.2f\tMax 1: %.2f\tMin 2: %.2f\tMax 2: %.2f\n\n',min1,max1,min2,max2);

        %If there is no overlap of the projections, there couldn't have
        %been a collision
        if(AgtB(min2, max1) == 1)
            fprintf('\nMin 2: %.2f\tMax 1: %.2f\nProjections do not overlap.\nNo collision.\n',min2,max1);
            %hasCollided = 0;
            return;
        end

        %Calculate the magnitude of the minimum translation vector
        %The MTV's magnitude is the value of the overlap of the projections
        %and the direction is the same as the current axis.
        if((max1 - min2) < magTrans)
            magTrans = max1 - min2;

            currentMTV = magTrans*currentAxis;
        end
    end

    %If there was a collision, these are the values of the collision
    MTV = currentMTV;

    %if the magnitude is 0 but there was a collision, the polygons are
    %tangential.
    if(norm(MTV) == 0)
        tangential = 1;
    else
        tangential = 0;
    end

    fprintf('\nCollision detected.\n');

    if(tangential ~= 1)
        fprintf('Magnitude of collision: %.2f\nDirection of collision: <%.2f,%.2f>\n',norm(MTV),MTV(1)/norm(MTV),MTV(2)/norm(MTV));
    else
        fprintf('The polygons are tangentially colliding.\n');
    end

end

%subtracts two vectors and gives the result
function [result] = SubVector(v1,v2)
    result = (v2-v1);
end

%returns the normal (orthogonal) vector
function [result] = NormalVector(v)
    result(1) = v(2);
    result(2) = v(1)*-1;
end

%normalizes a vector returning the unit vector of magnitude 1
function [result] = UnitVector(v)
    result = v/norm(v);
end

%Returns 1 if A > B and 0 is else
function [result] = AgtB(A,B)
    if(A > B)
        result = 1;
    else
        result = 0;
    end
end

%Retrieves the amount of faces and angle of rotation of polygons for each
%polygon
function [amtFaces1, amtFaces2, rot1, rot2, centerX1, centerX2, centerY1, centerY2, size1, size2, failure] = processInput()
    %Initialize values
    amtFaces1 = 0; amtFaces2 = 0; rot1 = 0; rot2 = 0; centerX1 = 0; centerX2 = 0; centerY1 = 0; centerY2 = 0; size1 = 0; size2 = 0; failure = 0;
    amtFaces1 = input('Enter the number of faces for polygon 1: ');
    if((mod(amtFaces1,1) ~= 0) || (amtFaces1 < 0))
        fprintf('\nNumber of faces must be a positive integer.\nPlease try again.\n\n');
        failure = 1;
        return;
    end
    rot1 = input('Enter the angle of rotation for polygon 1 (RADIANS): ');
    centerX1 = input('Enter the x coordinate for the center of polygon 1: ');
    centerY1 = input('Enter the y coordinate for the center of polygon 1: ');
    size1 = input('Enter the size of polygon 1: ');
    if(size1 <= 0)
        fprintf('\nSize must be a positive number greater than 0.\nPlease try again.\n\n');
        failure = 1;
        return;
    end

    fprintf('\n');

    amtFaces2 = input('Enter the number of faces for polygon 2: ');
    if((mod(amtFaces2,1) ~= 0) || (amtFaces1 < 0))
        fprintf('\nNumber of faces must be a positive integer.\nPlease try again.\n\n');
        failure = 1;
        return;
    end
    rot2 = input('Enter the angle of rotation for polygon 2 (RADIANS): ');
    centerX2 = input('Enter the x coordinate for the center of polygon 2: ');
    centerY2 = input('Enter the y coordinate for the center of polygon 2: ');
    size2 = input('Enter the size of polygon 2: ');
    if(size2 <= 0)
        fprintf('\nSize must be a positive number greater than 0.\nPlease try again.\n\n');
        failure = 1;
        return;
    end
end

%creates a polygons vertices from the arguments provided
function [vX] = createPolygonX(amtFace, rot, centerX, size)
    vX = zeros(amtFace,1);  %allocate the vertices ahead of time
    angleSeparation = (2*pi) / amtFace; %each vertex is separated by this angle, made from separating the unit circle into even subdivisions
    currentAngle = (pi/2);    %the first vertex will be at the top

    %create the vertices for each polygon
    for i = 1:1:amtFace
        vX(i) = centerX + (size * cos(currentAngle + rot));
        currentAngle = currentAngle + angleSeparation;
    end
end

function [vY] = createPolygonY(amtFace, rot, centerY, size)
    vY = zeros(amtFace,1);
    angleSeparation = (2*pi) / amtFace; %each vertex is separated by this angle, made from separating the unit circle into even subdivisions
    currentAngle = (pi/2);         %the current angle which is used to generate the vertices

    %create the vertices for each polygon
    for i = 1:1:amtFace
        vY(i) = centerY + (size * sin(currentAngle + rot));
        currentAngle = currentAngle + angleSeparation;
    end
end
