Rx = [1 0 0; 0 -1 0; 0 0 -1];
Rz = [0 1 0; -1 0 0; 0 0 1];

T1 = [0 1 0 -1.5; 1 0 0 0.5; 0 0 -1 3; 0 0 0 1];
T1 = inv(T1);


%% Tranformations from cubes to end-effector

%Tranformation from the table to the camera


thetaX1 = 180;
thetaZ1 = -90;
depthOrigin = 0.2; %change this according to hand measured depth 


rotationX_CamToTable = [1 0 0; 
                      0 cos(thetaX1) -sin(thetaX1);
                      0 sin(thetaX1) cos(thetaX1)];
                      
                  
translationX_CamToTable = [0;
                           0;
                           0];
                                         
                  
rotationZ_CamToTable = [cos(thetaZ1) -sin(thetaZ1) 0;
                        sin(thetaZ1) cos(thetaZ1) 0;
                        0 0 1];
                        
                    
translationZ_CamToTable = [0;
                           0;
                           depthOrigin];
                           

rotation_CamToTable = rotationX_CamToTable*rotationZ_CamToTable;


transformTableToCam = [(rotation_CamToTable) [0; 0; depthOrigin]
                        [0 0 0] 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Tranformation from the table to the end-effector

thetaX2 = 180;
thetaZ2 = -90; 
offset =          %measure and change value in X

rotationX_TableToEndEff = [1 0 0;
                         0 cos(thetaX2) -sin(thetaX2);
                         0 sin(thetaX2) cos(thetaX2)];

translationX_TableToEndEff = [offset;
                            0;
                            0];
                        
rotationZ_TableToEndEff = [cos(thetaZ2) -sin(thetaZ2) 0;
                         sin(thetaZ2) cos(thetaZ2) 0;
                         0 0 1];
                     
rotation_TableToEndEff = rotationX_TableToEndEff*rotationZ_TableToEndEff;

transformTableToEndEff = [(rotation_TableToEndEff) [offset; 0; 0]
                        [0 0 0] 1];
    

                         

                  
                  










