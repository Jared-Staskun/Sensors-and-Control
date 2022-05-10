%% Intialise Connection
rosshutdown
rosinit;

%% Initialise Dobot
fprintf('Intialising...\n');

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses');

default_pos = [0.2589,0,-0.0085];
ground_level = -0.0589;

%% Robot Safety Status
currentSafetyStatus = 0;

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
while(currentSafetyStatus ~=4)
    currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;
end

fprintf('Safety Loop Done.\n');

%Move dobot out of the way
out_of_way = [0.0053, -0.2205, 0.0324];
Move_End_Effector(out_of_way)
pause(3)

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");

[msg2] = receive(sub);
image = readImage(msg2);

%% Find Centriods

%find centre of red cube
extracted_red = extract_red(image);                     % Outputs binary (0,1) same size array with red as 1
red_centre = calculate_centroid(extracted_red);         % Outputs pixel coordinates of red cube centre
figure, imshow(extracted_red);                          % Show the extracted RED (RED pixels are white, every other pixel is black)

%find centre of green cube
extracted_green = extract_green(image);
green_centre = calculate_centroid(extracted_green);     % Outputs pixel coordinates of green cube centre
figure, imshow(extracted_green);

%find centre of yellow cube
extracted_yellow = extract_yellow(image);
yellow_centre = calculate_centroid(extracted_yellow);   %  Outputs pixel coordinates of yellow cube centre
figure, imshow(extracted_yellow);

%find centre of purple cube
extracted_purple = extract_purple(image);
purple_centre = calculate_centroid(extracted_purple);   %  Outputs pixel coordinates of purple cube centre
figure, imshow(extracted_purple);

%plot results
figure, imshow(image);
hold on

scatter(red_centre.Centroid(1), red_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(red_centre.Centroid(1) + 10, red_centre.Centroid(2),'RED','FontSize',14,'FontWeight','bold');

scatter(green_centre.Centroid(1), green_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(green_centre.Centroid(1) + 10, green_centre.Centroid(2),'GREEN','FontSize',14,'FontWeight','bold');

scatter(yellow_centre.Centroid(1), yellow_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(yellow_centre.Centroid(1) + 10, yellow_centre.Centroid(2),'YELLOW','FontSize',14,'FontWeight','bold');

scatter(purple_centre.Centroid(1), purple_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(purple_centre.Centroid(1) + 10, purple_centre.Centroid(2),'PURPLE','FontSize',14,'FontWeight','bold');

%% Calculate Coordinates

%%%% Calculate 3D Point Coordinates on Camera Frame

% Camera Parameters
 FL = (917.6252+915.7077)/2;
 PPx = 636.2969;
 PPy = 351.4231;

% % Move origin to principle point:
    % RED
        Red_X_L = red_centre.Centroid(1,1) - PPx;
        Red_Y_L = red_centre.Centroid(1,2) - PPy;
    % GREEN
        Green_X_L = green_centre.Centroid(1,1) - PPx;
        Green_Y_L = green_centre.Centroid(1,2) - PPy;
    % YELLOW
        Yellow_X_L = yellow_centre.Centroid(1,1) - PPx;
        Yellow_Y_L = yellow_centre.Centroid(1,2) - PPy;
    % PURPLE
        Purple_X_L = purple_centre.Centroid(1,1) - PPx;
        Purple_Y_L = purple_centre.Centroid(1,2) - PPy;

% Collect Depth (Z value) information from Camera (ROS-SUBSCRIBE???? OR MEASURE)
    Z_Cam_To_Block = 557/1000;         % Z = 557mm = 0.557m

% Calculate X and Y Coordinates using already known Z
    Red_X_Camera = (Z_Cam_To_Block*Red_X_L) / FL;
    Red_Y_Camera = (Z_Cam_To_Block*Red_Y_L) / FL;

    Green_X_Camera = (Z_Cam_To_Block*Green_X_L) / FL;
    Green_Y_Camera = (Z_Cam_To_Block*Green_Y_L) / FL;

    Yellow_X_Camera = (Z_Cam_To_Block*Yellow_X_L) / FL;
    Yellow_Y_Camera = (Z_Cam_To_Block*Yellow_Y_L) / FL;

    Purple_X_Camera = (Z_Cam_To_Block*Purple_X_L) / FL;
    Purple_Y_Camera = (Z_Cam_To_Block*Purple_Y_L) / FL;

% Store in 1 by 3 Matrix of Coordinates (x,y,z)
    RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Z_Cam_To_Block]
    GREEN_CamCoordinate = [Green_X_Camera Green_Y_Camera Z_Cam_To_Block]
    YELLOW_CamCoordinate = [Yellow_X_Camera Yellow_Y_Camera Z_Cam_To_Block]
    PURPLE_CamCoordinate = [Purple_X_Camera Purple_Y_Camera Z_Cam_To_Block]
    
%% Convert Cam Coordinates to Dobot Coordinates
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
    GroundToDobotCentre = 0.06;
    Block_Height = 0.0291; 
% Converts CamXYZ To DobotXYZ
    R_Dobot_10mmTOP = [(0.079+0.1905)-RED_CamCoordinate(1,2) (-1)*RED_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height-0.01];
    R_Dobot_0mmTOP = [(0.079+0.1905)-RED_CamCoordinate(1,2) (-1)*RED_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    G_Dobot_10mmTOP = [(0.079+0.1905)-GREEN_CamCoordinate(1,2) (-1)*GREEN_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height-0.01];
    G_Dobot_0mmTOP = [(0.079+0.1905)-GREEN_CamCoordinate(1,2) (-1)*GREEN_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    Y_Dobot_10mmTOP = [(0.079+0.1905)-YELLOW_CamCoordinate(1,2) (-1)*YELLOW_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height-0.01];
    Y_Dobot_0mmTOP = [(0.079+0.1905)-YELLOW_CamCoordinate(1,2) (-1)*YELLOW_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    P_Dobot_10mmTOP = [(0.079+0.1905)-PURPLE_CamCoordinate(1,2) (-1)*PURPLE_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height-0.01];
    P_Dobot_0mmTOP = [(0.079+0.1905)-PURPLE_CamCoordinate(1,2) (-1)*PURPLE_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];

% 

%% Movement
Origin = [0.2695, 0, 0];
Move_End_Effector(Origin)

%Move_End_Effector(default_pos)
Move_End_Effector(G_Dobot_10mmTOP)
Move_End_Effector(G_Dobot_0mmTOP)

%% Functions

function [BW_object] = extract_red(image)
%Isolate red objects and return black and white image of isolated objects
    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.947;
    channel1Max = 0.062;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.309;
    channel2Max = 1.000;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.691;
    channel3Max = 1.000;

    % Create mask based on chosen histogram thresholds
    sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ... 
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            % Channel 1 Min and Max are "OR"
            % Channel 2 Min and Max are "AND"
            % Channel 3 Min and Max are "AND"
    BW_object = sliderBW;
end

function [BW_object] = extract_green(image)
%Isolate green objects and return black and white image of isolated objects
    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.397;
    channel1Max = 0.500;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.543;
    channel2Max = 1.000;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.000;
    channel3Max = 1.000;

    % Create mask based on chosen histogram thresholds
    sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW_object = sliderBW;
end

function [BW_object] = extract_yellow(image)
%Isolate yellow objects and return black and white image of isolated objects
    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.102;
    channel1Max = 0.179;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.459;
    channel2Max = 0.890;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.000;
    channel3Max = 1.000;

    % Create mask based on chosen histogram thresholds
    sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW_object = sliderBW;
end

function [BW_object] = extract_purple(image)
%Isolate purple objects and return black and white image of isolated objects
    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.614;
    channel1Max = 0.699;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.083;
    channel2Max = 0.265;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.512;
    channel3Max = 0.787;

    % Create mask based on chosen histogram thresholds
    sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW_object = sliderBW;
end

function [object_centre] = calculate_centroid(BW_object)
%Calculate the coordinates of the pixel group of white pixels
target_object = bwareafilt(BW_object,1);
object_centre = regionprops(target_object,'centroid');
end