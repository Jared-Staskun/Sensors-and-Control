Rx = [1 0 0; 0 -1 0; 0 0 -1];
Rz = [0 1 0; -1 0 0; 0 0 1];

T1 = [0 1 0 -1.5; 1 0 0 0.5; 0 0 -1 3; 0 0 0 1];
T1 = inv(T1);


%% Tranformations from cubes to end-effector

%Tranformation from the floor of the table to the camera
ThetaX = 180;
ThetaZ = -90;


RotationX_CamToTable = [1 0 0 0; 
                      0 cos(ThetaX) -sin(ThetaX) 0;
                      0 sin(ThetaX) cos(ThetaX) 0;
                      0 0 0 1];
                  
TranslationX_CamToTable = [1 0 0 CubeCentreX;
                           0 1 0 0;
                           0 0 1 0;
                           0 0 0 1];
                                                 
TranslationY_CamToTable = [1 0 0 0;
                           0 1 0 CubeCentreY;
                           0 0 1 0;
                           0 0 0 1];                 
                  
RotationZ_CamToTable = [cos(ThetaZ) -sin(ThetaZ) 0 0;
                        sin(ThetaZ) cos(ThetaZ) 0 0;
                        0 0 1 0;
                        0 0 0 1];
                    
TranslationZ_CamToTable = [1 0 0 0;
                           0 1 0 0;
                           0 0 1 Depth;
                           0 0 0 1];

Transformation_CamToTable = [
                  
                  









