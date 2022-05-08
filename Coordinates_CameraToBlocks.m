%% Main
hold off
image = imread('test_image.png');
figure, imshow(image);                                  % Show the initial RGB image

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

%% Calculate 3D Point Coordinates on Camera Frame

% ROUND TO NEAREST INTEGER (PIXEL COORDINATES)
%   for c = 1:2
%       red_centre.Centroid(1,c) = round( red_centre.Centroid(1,c) );
%   end
%   for c = 1:2
%       green_centre.Centroid(1,c) = round( green_centre.Centroid(1,c) );
%   end
%   for c = 1:2
%       yellow_centre.Centroid(1,c) = round( yellow_centre.Centroid(1,c) );
%   end
%   for c = 1:2
%       purple_centre.Centroid(1,c) = round( purple_centre.Centroid(1,c) );
%   end

% Camera Parameters
 Camera_Focal_Length = 945.224;         % 945.224
 Camera_Principle_Point_X = 959.023;    % 959.023
 Camera_Principle_Point_Y = 532.926;    % 532.926

% Move origin to principle point:
    % RED
        Red_X_L = red_centre.Centroid(1,1) - Camera_Principle_Point_X;
        Red_Y_L = red_centre.Centroid(1,2) - Camera_Principle_Point_Y;
    % GREEN
        Green_X_L = green_centre.Centroid(1,1) - Camera_Principle_Point_X;
        Green_Y_L = green_centre.Centroid(1,2) - Camera_Principle_Point_Y;
    % YELLOW
        Yellow_X_L = yellow_centre.Centroid(1,1) - Camera_Principle_Point_X;
        Yellow_Y_L = yellow_centre.Centroid(1,2) - Camera_Principle_Point_Y;
    % PURPLE
        Purple_X_L = purple_centre.Centroid(1,1) - Camera_Principle_Point_X;
        Purple_Y_L = purple_centre.Centroid(1,2) - Camera_Principle_Point_Y;

% Collect Depth (Z value) information from Camera (ROS-SUBSCRIBE???? OR MEASURE)
    Red_Z_Camera = 0.2;         % Z = 20cm = 0.2m
    Green_Z_Camera = 0.2;
    Yellow_Z_Camera = 0.2;
    Purple_Z_Camera = 0.2;

% Calculate X and Y Coordinates using already known Z
    Red_X_Camera = (Red_Z_Camera*Red_X_L) / Camera_Focal_Length;
    Red_Y_Camera = (Red_Z_Camera*Red_Y_L) / Camera_Focal_Length;

    Green_X_Camera = (Green_Z_Camera*Green_X_L) / Camera_Focal_Length;
    Green_Y_Camera = (Green_Z_Camera*Green_Y_L) / Camera_Focal_Length;

    Yellow_X_Camera = (Yellow_Z_Camera*Yellow_X_L) / Camera_Focal_Length;
    Yellow_Y_Camera = (Yellow_Z_Camera*Yellow_Y_L) / Camera_Focal_Length;

    Purple_X_Camera = (Purple_Z_Camera*Purple_X_L) / Camera_Focal_Length;
    Purple_Y_Camera = (Purple_Z_Camera*Purple_Y_L) / Camera_Focal_Length;

% Store in 1 by 3 Matrix of Coordinates (x,y,z)
    RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Red_Z_Camera]
    GREEN_CamCoordinate = [Green_X_Camera Green_Y_Camera Green_Z_Camera]
    YELLOW_CamCoordinate = [Yellow_X_Camera Yellow_Y_Camera Yellow_Z_Camera]
    PURPLE_CamCoordinate = [Purple_X_Camera Purple_Y_Camera Purple_Z_Camera]

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