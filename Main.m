%% Intialise Connection
rosshutdown  % Turns off the current running ROS core
rosinit;  %  Starts a ROS core

%% Initialise Dobot
fprintf('Intialising...\n');      % A string to indicate that intialisation is commencing in command window

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');    % Publisher to publish the DOBOT safety status

safetyStateMsg.Data = 2;    % Prepare message to initialise and home the dobot
send(safetyStatePublisher,safetyStateMsg);  % Send the message

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');   % Publisher to turn the suction gripper on and off
[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose'); % Publiser to move the end effector to set positions
endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Subscriber to get current end effector pose

default_pos = [0.2589,0,-0.0085];   % The default pose the robot moves to after initialisation
ground_level = -0.0589;             % The z value of the end effector when it contacts the ground

%% Robot Safety Status
currentSafetyStatus = 0;    % Ensure the safety status has been reset

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');    % Subscriber to get safety status of the robot
pause(2); %Allow some time for MATLAB to start the subscriber
while(currentSafetyStatus ~=4)      % Safety status 4 => Operating
    currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;        % Wait in the while loop until the robot is ready and in the operating state
end

fprintf('Safety Loop Done.\n');     % Print message to console to confirm that the robot is done initialising

%Move dobot out of the way
out_of_way = [0.0053, -0.2205, 0.0324];     % Position of end effector that moves the robot arm out of the way of the camera
Move_End_Effector(out_of_way)               % Move arm out of the camera's view
pause(3)                                    % Give some time for the arm to get out the way

%% Initialise the Camera

sub = rossubscriber("camera/color/image_raw");    % Create subscriber to obtain data from '/color/mage_raw' camera topic

[msg2] = receive(sub);  % Stores data from subscriber in 'msg2'
image = readImage(msg2);  % Converts and stores data as RGB image

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

%find centre of blue cube
extracted_blue = extract_blue(image);
blue_centre = calculate_centroid(extracted_blue);   %  Outputs pixel coordinates of blue cube centre
figure, imshow(extracted_blue);

%plot results
figure, imshow(image);   % plots RGB image
hold on

scatter(red_centre.Centroid(1), red_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');   % Displays the cube centroid in pixels with black text
text(red_centre.Centroid(1) + 10, red_centre.Centroid(2),'RED','FontSize',14,'FontWeight','bold');

scatter(green_centre.Centroid(1), green_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(green_centre.Centroid(1) + 10, green_centre.Centroid(2),'GREEN','FontSize',14,'FontWeight','bold');

scatter(yellow_centre.Centroid(1), yellow_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(yellow_centre.Centroid(1) + 10, yellow_centre.Centroid(2),'YELLOW','FontSize',14,'FontWeight','bold');

scatter(blue_centre.Centroid(1), blue_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
text(blue_centre.Centroid(1) + 10, blue_centre.Centroid(2),'BLUE','FontSize',14,'FontWeight','bold');

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
    % blue
        blue_X_L = blue_centre.Centroid(1,1) - PPx;
        blue_Y_L = blue_centre.Centroid(1,2) - PPy;

% Collect Depth (Z value) information from Camera (ROS-SUBSCRIBE???? OR MEASURE)
    Z_Cam_To_Block = 557/1000;         % Z = 557mm = 0.557m

% Calculate X and Y Coordinates using already known Z
    Red_X_Camera = (Z_Cam_To_Block*Red_X_L) / FL;
    Red_Y_Camera = (Z_Cam_To_Block*Red_Y_L) / FL;

    Green_X_Camera = (Z_Cam_To_Block*Green_X_L) / FL;
    Green_Y_Camera = (Z_Cam_To_Block*Green_Y_L) / FL;

    Yellow_X_Camera = (Z_Cam_To_Block*Yellow_X_L) / FL;
    Yellow_Y_Camera = (Z_Cam_To_Block*Yellow_Y_L) / FL;

    blue_X_Camera = (Z_Cam_To_Block*blue_X_L) / FL;
    blue_Y_Camera = (Z_Cam_To_Block*blue_Y_L) / FL;

% Store in 1 by 3 Matrix of Coordinates (x,y,z)
    RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Z_Cam_To_Block]
    GREEN_CamCoordinate = [Green_X_Camera Green_Y_Camera Z_Cam_To_Block]
    YELLOW_CamCoordinate = [Yellow_X_Camera Yellow_Y_Camera Z_Cam_To_Block]
    blue_CamCoordinate = [blue_X_Camera blue_Y_Camera Z_Cam_To_Block]
    
%% Convert Cam Coordinates to Dobot Coordinates
% Know these 1 by 3 Matrix Coordinates from Camera To Centroid of Block
    % RED_CamCoordinate - 1x3 MATRIX (Rx Ry Rz)
    % GREEN_CamCoordinate - 1x3 MATRIX (Gx Gy Gz)
    % YELLOW_CamCoordinate - 1x3 MATRIX (Yx Yy Yz)
    % blue_CamCoordinate - 1x3 MATRIX (Px Py Pz)
% Dobot Position at Camera Origin
    % X = 0.079 + 0.1905;
    % Y = 0;
    % Z = 0;
% Dobot Parameters
    % AT Z = 0, Dobot origin is 6cm above ground surface
    GroundToDobotCentre = 0.057;
    Block_Height = 0.0260;
    tavel_height = 0.0900;
% Converts CamXYZ To DobotXYZ
    R_Dobot_10mmTOP = [0.26925-RED_CamCoordinate(1,2) (-1)*RED_CamCoordinate(1,1) 0.08];
    R_Dobot_0mmTOP = [0.26925-RED_CamCoordinate(1,2) (-1)*RED_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    G_Dobot_10mmTOP = [0.26925-GREEN_CamCoordinate(1,2) (-1)*GREEN_CamCoordinate(1,1) 0.8];
    G_Dobot_0mmTOP = [0.26925-GREEN_CamCoordinate(1,2) (-1)*GREEN_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    Y_Dobot_10mmTOP = [0.26925-YELLOW_CamCoordinate(1,2) (-1)*YELLOW_CamCoordinate(1,1) 0.08];
    Y_Dobot_0mmTOP = [0.26925-YELLOW_CamCoordinate(1,2) (-1)*YELLOW_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];
    
    B_Dobot_10mmTOP = [0.26925-blue_CamCoordinate(1,2) (-1)*blue_CamCoordinate(1,1) 0.08];
    B_Dobot_0mmTOP = [0.26925-blue_CamCoordinate(1,2) (-1)*blue_CamCoordinate(1,1) 0-GroundToDobotCentre+Block_Height];

% 

%% Movement
block_height_factor = 0.0300;   % Approximate height of the blocks used in the demo + a small gap

Origin = [0.26925, 0, 0];
Move_End_Effector(Origin)
pause(3)

%Red Block
Move_End_Effector(R_Dobot_10mmTOP)  % Move directly above the block before appoaching the block
pause(3)
Move_End_Effector(R_Dobot_0mmTOP)
pause(3)
tool_state(1);  % Suction ON
pause(3)
Move_End_Effector(R_Dobot_10mmTOP)
pause(3)

Drop = [0.26925, 0, (ground_level + block_height_factor * 1)];   % Move to the position to build the tower. Z is the ground level height + the height of 1 block
Move_End_Effector(Drop)
pause(3)
tool_state(0);  % Suction OFF

%Yellow Block - Very similar to process to red block
Move_End_Effector(Y_Dobot_10mmTOP)
pause(3)
Move_End_Effector(Y_Dobot_0mmTOP)
pause(3)
tool_state(1);
pause(3)
Move_End_Effector(Y_Dobot_10mmTOP)
pause(3)

Drop = [0.26925, 0, (ground_level + block_height_factor * 2)]; % Block height factor multiplied by 2 as there is now 2 blocks between end effector and ground
Move_End_Effector(Drop)
pause(3)
tool_state(0);

%Green Block
Move_End_Effector(G_Dobot_10mmTOP)
pause(3)
Move_End_Effector(G_Dobot_0mmTOP)
pause(3)
tool_state(1);
pause(3)
Move_End_Effector(G_Dobot_10mmTOP)
pause(3)

Drop = [0.26925, 0, (ground_level + block_height_factor * 3)];
Move_End_Effector(Drop)
pause(3)
tool_state(0);

%Blue Block
Move_End_Effector(B_Dobot_10mmTOP)  
pause(3)
Move_End_Effector(B_Dobot_0mmTOP)
pause(3)
tool_state(1);
pause(3)
Move_End_Effector(B_Dobot_10mmTOP)
pause(3)

Drop = [0.26925, 0, (ground_level + block_height_factor * 4)];
Move_End_Effector(Drop)
pause(3)
tool_state(0);

%% Knock it down

% Code that moves end-effector to the side of the stacked blocks and knocks them over
pos1 = [0.2687 -0.0758 0.08];
pos2 = [0.2719 -0.0676 -0.0199];
pos3 = [0.2689 0.0409 -0.0089];   

Move_End_Effector(pos1)
pause(5)
Move_End_Effector(pos2)
pause(1)
Move_End_Effector(pos3)

%% Functions

function [BW_object] = extract_red(image)
%Isolate red objects and return black and white image of isolated objects
    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.986;
    channel1Max = 0.039;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.424;
    channel2Max = 1.000;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.000;
    channel3Max = 1.000;

    % Create mask based on chosen histogram thresholds
    sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
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

function [BW_object] = extract_blue(image)
    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.518;
    channel1Max = 0.559;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.399;
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

function [object_centre] = calculate_centroid(BW_object)
%Calculate the coordinates of the pixel group of white pixels
    target_object = bwareafilt(BW_object,1);                    % Isolates the biggest group of white pixels in a black and while image
    object_centre = regionprops(target_object,'centroid');      % Find the centriod of the isolated pixels from the previous function
end