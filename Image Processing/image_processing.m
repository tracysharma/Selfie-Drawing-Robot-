%% Image Processing
% This code takes an image (preferably of a face) and performs bicubic
% vectorisation on it.
% Output is a csv file with x and y coordinates of the image features.
clear all; 
close all; 
clc; 

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


I = imread(uigetfile('*.*', 'Select an Image'));    % Allow selection of any file type and immediately read the image   
                                                    % The image need to be in the same folder as this MATLAB program

% I = imread('captured_image.png'); % Alternatively use specific image 


%% Read an image file 

figure; 
imshow(I);  % Show the image


%% Edge detection 

BW = edge(rgb2gray(I),'Canny',[0.0813 0.1281]); % Returns a binary image BW containing 1s where the function finds edges 
                                                % in the grayscale or binary image I and 0s elsewhere.
                                                % Returns all edges that are stronger/weaker than low and high thresholds.
                                                % The Canny method is less likely than the other methods to be fooled by noise, and more likely to detect true weak edges.

figure, imshow(BW);
%% Skeletonization/Segmentation

BWseg = bwmorph(BW,'skel',Inf); % Applies a specific morphological operation to the binary image BW.
                                % Inf makes the operation repeat until the image no longer changes.
                                % With n = Inf, remove pixels on the boundaries of objects without allowing objects to break apart. 
                                % The pixels remaining make up the image skeleton.
                                % Morphological operations are image processing techniques that deal with the shape or structure of objects in an image. 
                                % They work mainly with binary images, where pixels are either black (0) or white (1).
figure, imshow(BWseg);

%% Vectorization

% Nerest Neighbour Vectorisation

% vectors = nearestNeighborVectorization(BWseg);

% Export vectors to CSV file
% filename = 'vectors.csv';
% writematrix(vectors, filename, 'Delimiter', ',');

% Bicubic Vectorisation

bicubic_vec = BicubicVectorization(BWseg, 'bicubic'); % Perform bicubic vectorisation

filename = 'bicubic_vectors.csv';
writematrix(bicubic_vec, filename, 'Delimiter', ','); % Saves output as csv file

%% Visualize Image from CSV
% Nearest Neigbour Vectorisation

% Read vectors from CSV
% vectors = readmatrix('vectors.csv');
% 
% % Plot
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

% Bicubic Vectorisation

% Read in CSV file
vectors = readmatrix('bicubic_vectors.csv');

% Plot
figure;
plot(vectors(:,1), vectors(:,2), 'o'); % Plot points from csv file to double check
title('Visual Representation of Bicubic Vectors');
xlabel('X Coordinate');
ylabel('Y Coordinate');
axis equal;
grid on;

%% Extract x and y points (if using nearest neighbour)
% Assume 'vectors' is the output from the nearest neighbor vectorization
% vectors = [y1, x1, y2, x2]; this is typical format after vectorization

% Convert this to a simple list of unique points
% This might involve reshaping the array and then using unique to filter duplicates
points = reshape(vectors.', 2, []).';  % Reshape and transpose to get all points in rows
unique_points = unique(points, 'rows');  % Optional: remove duplicate points

% Export to CSV
filename = 'xypoints_after_vect.csv';
writematrix(unique_points, filename);

%% Visualise Image from CSV (if using nearest neighbour)
points = readmatrix('xypoints_after_vect.csv');

% Plotting the points
figure;
plot(points(:,1), points(:,2), 'o'); % Plot points as circles
title('Visual Representation of Points');
xlabel('X Coordinate');
ylabel('Y Coordinate');
axis equal; % Ensure equal scaling on both axes
grid on; % Turn on the grid for easier visualization

%% Uniform Downsampling Example
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


