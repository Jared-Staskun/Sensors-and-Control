%% Intialise
rosinit;

sub = rossubscriber("camera/color/image_raw");

[msg2] = receive(sub);
image = readImage(msg2);

%% Main

% while(1)
[msg2] = receive(sub);
image = readImage(msg2);

hold off

% %find centre of red cube
extracted_red = extract_red(image);
red_centre = calculate_centroid(extracted_red);

%find centre of green cube
extracted_green = extract_green(image);
green_centre = calculate_centroid(extracted_green);

%find centre of yellow cube
extracted_yellow = extract_yellow(image);
yellow_centre = calculate_centroid(extracted_yellow);

%find centre of purple cube
extracted_purple = extract_purple(image);
purple_centre = calculate_centroid(extracted_purple);

%plot results
imshow(image);
hold on

scatter(red_centre.Centroid(1), red_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
red_u = num2str(round(red_centre.Centroid(1)));
red_v = num2str(round(red_centre.Centroid(2)));
red_info = append(red_u,", ",red_v);
text(red_centre.Centroid(1) + 10, red_centre.Centroid(2), 'red','FontSize',14,'FontWeight','bold');
text(red_centre.Centroid(1) + 10, red_centre.Centroid(2) + 25, red_info,'FontSize',14,'FontWeight','bold');

scatter(green_centre.Centroid(1), green_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
green_u = num2str(round(green_centre.Centroid(1)));
green_v = num2str(round(green_centre.Centroid(2)));
green_info = append(green_u,", ",green_v);
text(green_centre.Centroid(1) + 10, green_centre.Centroid(2) + 25, green_info,'FontSize',14,'FontWeight','bold');
text(green_centre.Centroid(1) + 10, green_centre.Centroid(2),'green','FontSize',14,'FontWeight','bold');

scatter(yellow_centre.Centroid(1), yellow_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
yellow_u = num2str(round(yellow_centre.Centroid(1)));
yellow_v = num2str(round(yellow_centre.Centroid(2)));
yellow_info = append(yellow_u,", ",yellow_v);
text(yellow_centre.Centroid(1) + 10, yellow_centre.Centroid(2) + 25, yellow_info,'FontSize',14,'FontWeight','bold');
text(yellow_centre.Centroid(1) + 10, yellow_centre.Centroid(2),'yellow','FontSize',14,'FontWeight','bold');

scatter(purple_centre.Centroid(1), purple_centre.Centroid(2),'MarkerFaceColor','black','MarkerEdgeColor','black');
purple_u = num2str(round(purple_centre.Centroid(1)));
purple_v = num2str(round(purple_centre.Centroid(2)));
purlpe_info = append(purple_u,", ",purple_v);
text(purple_centre.Centroid(1) + 10, purple_centre.Centroid(2) + 25, purlpe_info,'FontSize',14,'FontWeight','bold');
text(purple_centre.Centroid(1) + 10, purple_centre.Centroid(2),'purple','FontSize',14,'FontWeight','bold');

% end

%% Shutdown

rosshutdown;

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
taget_object = bwareafilt(BW_object,1);
object_centre = regionprops(taget_object,'centroid');
end
