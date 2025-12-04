function cubeFlag = isCubeCollision(cubeInfo,nearCoor,newCoor,step)
% The function is used to detect if the new path point collides with all the obstacles
 
cubeFlag = 0;

    for k1 = 1:size(cubeInfo,1)
        % The side length of the obstacle is expanded to prevent collision between the drone's body and the obstacle.
        xMin = cubeInfo(k1,1)-2;
        xMax = cubeInfo(k1,2)+2;
        yMin = cubeInfo(k1,3)-2;
        yMax = cubeInfo(k1,4)+2;
        zMin = cubeInfo(k1,5);
        zMax = cubeInfo(k1,6)+2;
        
        for k2 = 0:step/100:step
            deltaX = newCoor(1) - nearCoor(1);
            deltaY = newCoor(2) - nearCoor(2);
            deltaZ = newCoor(3) - nearCoor(3);
 
            r = sqrt(deltaX^2+deltaY^2+deltaZ^2);
            fai = atan2(deltaY,deltaX);
            theta = acos(deltaZ/r);
 
            x = k2*sin(theta)*cos(fai);
            y = k2*sin(theta)*sin(fai);
            z = k2*cos(theta);
            
            checkPoint = [x+nearCoor(1),y+nearCoor(2),z+nearCoor(3)];
            
            if (xMin<checkPoint(1) && checkPoint(1) < xMax) && (yMin<checkPoint(2) && checkPoint(2) < yMax) && (zMin<checkPoint(3) && checkPoint(3) < zMax)
                cubeFlag = 1;
                return;
            end
        end
        
    end
end