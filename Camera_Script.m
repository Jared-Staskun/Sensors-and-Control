rosshutdown
rosinit
clear all
close all

%player = VideoReader();

sub = rossubscriber("camera/color/image_raw");
%, 'DataFormat', 'struct'); %,r @(~, msg) processRealsenseMsg(msg, player));

%sub = rossubscriber('/chatter',@exampleHelperROSChatterCallback,'DataFormat','struct');
%vid = videoinput('winvideo'); 
%img = getsnapshot(vid);




[msg2] = receive(sub);
image = readImage(msg2);
%imshow(image)

%% Red

% Convert RGB image to chosen color space
I = rgb2hsv(test_image);

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
BW = sliderBW;
imshow(BW)

%% Green


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
BW = sliderBW;
imshow(BW)


%%

%BW_gray = rgb2gray(BW);
%cornerPoints = detectHarrisFeatures(BW,'MinQuality', 0.2); -> Not used to calculate centriod

hold off

%Filters for biggest group of white pixels and then calculates centriod
taget_object = bwareafilt(BW,1);
object_centre = regionprops(taget_object,'centroid');

%
imshow(taget_object)
hold on
plot(cornerPoints)
plot(object_centre.Centroid(1),object_centre.Centroid(2),'Color','red')

function caminfoCallback (hObject);

end

function processRealsenseMsg(msg, player)
    xyz = rosReadXYZ(msg);
    rgb = rosReadRGB(msg);
    view(player, xyz, rgb)
end



    

