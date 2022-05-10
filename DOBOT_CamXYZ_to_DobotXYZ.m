% Know these 1 by 3 Matrix Coordinates from Camera To Centroid of Block
    % RED_CamCoordinate - 1x3 MATRIX (Rx Ry Rz)
    % GREEN_CamCoordinate - 1x3 MATRIX (Gx Gy Gz)
    % YELLOW_CamCoordinate - 1x3 MATRIX (Yx Yy Yz)
    % PURPLE_CamCoordinate - 1x3 MATRIX (Px Py Pz)
% Dobot Position at Camera Origin
    % X = 0.079 + 0.1905;
    % Y = 0;
    % Z = 0;
% Dobot Parameters
    % AT Z = 0, Dobot origin is 6cm above ground surface
    GroundToDobotCentre = 0.6;
    Block_Height = 0.3;
% Converts CamXYZ To DobotXYZ
    R_Dobot = [(0.079+0.1905)-RED_CamCoordinate(1,2) (-1)*RED_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    G_Dobot = [(0.079+0.1905)-GREEN_CamCoordinate(1,2) (-1)*GREEN_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    Y_Dobot = [(0.079+0.1905)-YELLOW_CamCoordinate(1,2) (-1)*YELLOW_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    P_Dobot = [(0.079+0.1905)-PURPLE_CamCoordinate(1,2) (-1)*PURPLE_CamCoordinate(1,1) 0];

% 