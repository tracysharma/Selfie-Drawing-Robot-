%% Image Processing

%Enter the origin (by going to the centre of the paper then write down x and y value, ScaleValue and LiftPenHeight, 
%Select a file, change file type to ALL FILES and select image located within same folder
%Output will go to test.csv

%% Initialization 
clear all; 
close all; 
clc; 
rng('default');

ScaleValue = 300;  %scale value for x and y, default scale of 1 have    x and y value around 0.3
LiftPenHeight = 10;  %set value for z which is the hight we lift pen (in mm)

DistanceWhichLiftPen = 0.002 * ScaleValue;   %suggested ti not change this value (if want changed, change only 0.002)
                                         %if two points are further than this distance then we will lift pen (give z value) to the end and start point

origin = [144 134]; % origion of the graph, write as [x y]; set the origin (by having the robot arm going to the center of the paper and then writing down the x and y value) 
delta = 0.001; % the distance apart we want record (the smaller the more detailed drawing is)

boundary = 18;    %we can change to a different value but this works fine

ZOffset = 113.215;  %we can enter the offset value for z here, if enter -100 then all z value will be 100 less

% Yaw = -176.629;  %W value, for robot arm to hold the pen
% Roll = -2.447;   %P value 
% Pitch = 112.57;  %R value

%% Read in ROS image and save in folder
% Create a ROS master node
rosshutdown;
rosinit;

% Subscribe to the USB camera topic
usbCamSub = rossubscriber('/usb_cam/image_raw');

% Create a loop to continuously read images from the USB camera
while true
    % Receive the image message
    imgMsg = receive(usbCamSub);
    
    % Convert the ROS image message to MATLAB format
    img = readImage(imgMsg);

    % Display the image
    if isempty(img)
        display('No image data received')
    else
        imshow(img);
        drawnow;
    end
    
    % Check if the user has pressed the Enter key
    if waitforbuttonpress == 1 && strcmp(get(gcf, 'CurrentKey'), 'return')
        % Save the image as a PNG file
        imwrite(img, 'captured_image.png');
        disp('Image saved as captured_image.png');
        break;
    end
end

rosshutdown;

%% Acquire an image from user selection


I = imread(uigetfile('*.*', 'Select an Image'));  % Allow selection of any file type and immediately read the image   %the image need to be in the same folder as this MATLAB program
                         %change to ALL FILES, then can then you can see images and select

% I = imread('captured_image.png');  %alternatively use specific image 


%% Read an image file 

figure; 
imshow(I);  %show the image


%% Edge detection 

BW = edge(rgb2gray(I),'Canny',[0.0813 0.1281]); %0.0813,0.1281
                                                  %returns a binary image BW containing 1s where the function finds edges 
                                                  % in the grayscale or binary image I and 0s elsewhere.
                                                  %returns all edges that are stronger than threshold
                                                  %the Canny method is less likely than the other methods to be fooled by noise, and more likely to detect true weak edges.

figure, imshow(BW);
%% Skeletonization/Segmentation

BWseg = bwmorph(BW,'skel',Inf); %applies a specific morphological operation to the binary image BW.
                                %Inf makes the operation repeat until the image no longer changes.
                                %With n = Inf, remove pixels on the boundaries of objects without allowing objects to break apart. 
                                % The pixels remaining make up the image skeleton. This option preserves the Euler number.
                                %Morphological operations are image processing techniques that deal with the shape or structure of objects in an image. 
                                %They work mainly with binary images, where pixels are either black (0) or white (1).
figure, imshow(BWseg);

%% Nearest Neighbor Vectorization
% vectors = nearestNeighborVectorization(BWseg);
bicubic_vec = BicubicVectorization(BWseg, 'bicubic');

% Export vectors to CSV file
% filename = 'vectors.csv';
% writematrix(vectors, filename, 'Delimiter', ',');

filename = 'bicubic_vectors.csv';
writematrix(bicubic_vec, filename, 'Delimiter', ',');

% % Optionally, you can visualize the vectorized result
% figure;
% imshow(I);  % Show original image
% hold on;
% for i = 1:size(vectors, 1)
%     if vectors(i,1) == vectors(i,3)  % Horizontal line
%         plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'r', 'LineWidth', 2);
%     else  % Vertical line
%         plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'b', 'LineWidth', 2);
%     end
% end
% hold off;

%% Visualize Vectors from CSV
% Read vectors from CSV
% vectors = readmatrix('vectors.csv');
% 
% % Create a blank figure for plotting vectors
% figure;
% axis equal;
% hold on;
% title('Vector Visualization');
% xlabel('Column Index');
% ylabel('Row Index');
% for i = 1:size(vectors, 1)
%     if vectors(i,1) == vectors(i,3)  % Horizontal line
%         plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'r', 'LineWidth', 2);
%     else  % Vertical line
%         plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'b', 'LineWidth', 2);
%     end
% end
% hold off;

vectors = readmatrix('bicubic_vectors.csv');

% Create a blank figure for plotting vectors
figure;
plot(vectors(:,1), vectors(:,2), 'o');
title('Visual Representation of Bicubic Vectors');
xlabel('X Coordinate');
ylabel('Y Coordinate');
axis equal;
grid on;

%% % Assume 'vectors' is the output from the nearest neighbor vectorization
% vectors = [y1, x1, y2, x2]; this is typical format after vectorization

% Optionally convert this to a simple list of unique points if needed
% This might involve reshaping the array and then using unique to filter duplicates
points = reshape(vectors.', 2, []).';  % Reshape and transpose to get all points in rows
unique_points = unique(points, 'rows');  % Optional: remove duplicate points

% Export to CSV
filename = 'xypoints_after_vect.csv';
writematrix(unique_points, filename);

%% % Load the points from CSV
points = readmatrix('xypoints_after_vect.csv');

% Plotting the points
figure;
plot(points(:,1), points(:,2), 'o'); % Plot points as circles
title('Visual Representation of Points');
xlabel('X Coordinate');
ylabel('Y Coordinate');
axis equal; % Ensure equal scaling on both axes
grid on; % Turn on the grid for easier visualization

%% Uniform Downsampling
% Load the original points
points = readmatrix('xypoints_after_vect.csv');

% Downsample the points by selecting every 10th point
downsampled_points = points(1:10:end, :);

% Plotting the downsampled points
figure;
plot(downsampled_points(:,1), downsampled_points(:,2), 'o');
title('Visual Representation of Downsampled Points');
xlabel('X Coordinate');
ylabel('Y Coordinate');
axis equal;
grid on;


