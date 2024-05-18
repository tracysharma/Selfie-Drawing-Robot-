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
    imshow(img);
    
    % Check if the user has pressed the Enter key
    if waitforbuttonpress == 1 && strcmp(get(gcf, 'CurrentKey'), 'return')
        % Save the image as a PNG file
        imwrite(img, 'captured_image.png');
        disp('Image saved as captured_image.png');
    end
end



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
vectors = nearestNeighborVectorization(BWseg);

% Export vectors to CSV file
filename = 'vectors.csv';
writematrix(vectors, filename, 'Delimiter', ',');

% Optionally, you can visualize the vectorized result
figure;
imshow(I);  % Show original image
hold on;
for i = 1:size(vectors, 1)
    if vectors(i,1) == vectors(i,3)  % Horizontal line
        plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'r', 'LineWidth', 2);
    else  % Vertical line
        plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'b', 'LineWidth', 2);
    end
end
hold off;

%% Visualize Vectors from CSV
% Read vectors from CSV
vectors = readmatrix('vectors.csv');

% Create a blank figure for plotting vectors
figure;
axis equal;
hold on;
title('Vector Visualization');
xlabel('Column Index');
ylabel('Row Index');
for i = 1:size(vectors, 1)
    if vectors(i,1) == vectors(i,3)  % Horizontal line
        plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'r', 'LineWidth', 2);
    else  % Vertical line
        plot([vectors(i,2), vectors(i,4)], [vectors(i,1), vectors(i,3)], 'b', 'LineWidth', 2);
    end
end
hold off;


% %% Detect start and end points 
% 
% [B,L] = bwboundaries(BWseg,'noholes'); %This function finds the boundaries of objects in the binary image BWseg.
% imshow(label2rgb(L, @jet, [.5 .5 .5])) %colorise the labels using the 'jet' colourmap, and the background colour is set to grey.
% hold on
% for k = 1:length(B)
%    boundary = B{k};
%    plot(boundary(:,2), boundary(:,1), 'LineWidth', 3)
% end
% 
% 
% %% Find the end point 
% 
% edgeind = find(all(circshift(boundary,1) == circshift(boundary,-1),2),1); %circularly shift the rows of the boundary matrix by one position to the right and left, respectively.
%                                                                         %Checks if all elements in the two shifted matrices are equal row-wise
%                                                                         %edgeind will contain the index of the first row where the points in boundary 
%                                                                         %are the same as the points in the next row, indicating a circular structure. 
%                                                                         %This index is likely considered the "end point."
% %% Sort the points 
% 
% boundary = circshift(boundary,-edgeind+1); %circularly shifts the rows of boundary to bring the identified "end point" to the first row.
% boundary = boundary(1:ceil(end/2),:); %This selects the first half of the rows in boundary, effectively sorting and truncating the points.
% 
% %% Create path for each edge 
% 
% for i = 1:length(B)                                                         %he loop iterates over each boundary (B{i}), finds the end point, sorts the points, and then updates the boundary. 
%                                                                             %After processing all boundaries, it overlays the modified boundaries on the labeled image. 
%     boundary = B{i};
%     edgeind = find(all(circshift(boundary,1) == circshift(boundary,-1),2),1);
% 
%     if ~isempty(edgeind)
%         boundary = circshift(boundary,-edgeind+1);
%         boundary = boundary(1:ceil(end/2),:);
%     end
%     B{i} = boundary;
% end
% imshow(label2rgb(L, @jet, [.5 .5 .5]))
% hold on
% for k = 1:length(B)
%    boundary = B{k};
%    plot(boundary(:,2), boundary(:,1), 'LineWidth', 3)
% end
% 
% %% Convert to 3D coordinate 
% %convert 2D coordinates to 3D coordinates 
% B2 = B;
% figure;
% for i = 1:length(B) %For each boundary, it calculates the corresponding 3D coordinates (bx, by, and bz) based on the X and Y coordinates of the boundary points.
%     b = B{i};
%     bx = -b(:,2) * delta * ScaleValue;  
%     by = b(:,1) * delta * ScaleValue;   
%     bz = zeros(length(bx),1); %The Z-coordinate (bz) is set to zero and will need to be adjusted later
%     B2{i} = [bx by bz];
%     plot3(bx,by,bz); 
%     hold on;
% end
% grid on;
% 
% %% Find the center point of the graph, then shift to 0,0 then shift to our center point
% 
% XYZmatrix = cell2mat(B2);   %if we don't convert then we need write using writecell and the output is a mess
% %therefore we convert it to matrix then export, the result is much cleaner
% 
% XYZmatrixRowSize = size(XYZmatrix,1);   %we get matrix's first dimension (row) size used later for for loop
% 
% SumOfEachColumn = sum(XYZmatrix,1);  %S = sum(A,dim) returns the sum along dimension dim. For example, if A is a matrix, then sum(A,1) is a column vector containing the sum of each column
% OldCenterPointX = SumOfEachColumn(1) / size(XYZmatrix,1);  %size(XYZmatrix,1) is number of rows of XYZ matrix
% OldCenterPointY = SumOfEachColumn(2) / size(XYZmatrix,1);
% %first column is sum of x values, second column is sum of y values
% 
% for iii = 1:XYZmatrixRowSize
%       XYZmatrix(iii,1) = XYZmatrix(iii,1) - OldCenterPointX+origin(1);   
%       XYZmatrix(iii,2) = XYZmatrix(iii,2) - OldCenterPointY+origin(2);  
% end
% 
% plot3(XYZmatrix(:,1),XYZmatrix(:,2),XYZmatrix(:,3)); 
% 
% %% Edit z axis if we start draw new line (like human drawing not connected lines)
% 
%     previousX = 0;   %after we store bx and by, we check it with new bx and by value, first time set to 0
%     previousY = 0;
% 
% for ii = 1:XYZmatrixRowSize
% 
%     ThisX = XYZmatrix(ii,1);   %extract the first folumn (which is x) of every XYZmatrix using for loop
%     ThisY = XYZmatrix(ii,2);    %extract the second folumn (which is y) of every XYZmatrix using for loop
% 
% 
%    if abs(ThisX-previousX) > DistanceWhichLiftPen || abs(ThisY-previousY) > DistanceWhichLiftPen  
%        %if the move distence is too much between two points then lift pen to move robot arm there
%       XYZmatrix(ii,3) = ZOffset + LiftPenHeight;   %we change the z value here for how high the pen lift
%    else
%       XYZmatrix(ii,3) = ZOffset + 0;  %no lifting means drawing
%    end
% 
%     previousX = ThisX;   %after we store bx and by, we set it as new previous value, which will be compared when run for loop again
%     previousY = ThisY;
% end
% 
% 
% %%  Replace the previous stroke (end stroke of previous drawing)'s z axis to a higher value because we want lift at end and beginning of draw line
% 
%  %the end point of previous stroke need lift we need lift up pen already,
%  %so replace previous stroke z value to 20
% 
% [NewStrokeRows,~] = find(XYZmatrix(:,3) == ZOffset + LiftPenHeight);    % A(:,3) means: all rows third column, which is z value, equal to LiftPenHeight, then we locate the row
% % [row,col] = find(___) returns the row and column subscripts of each nonzero element in array X using any of the input arguments in previous syntaxes.
% %we fild the third column which has value of LiftPenHeight, the position is
% %stored in array NewStrokeRows
% 
% for xx = 1:numel(NewStrokeRows)    %n = numel( A ) returns the number of elements, n , in array A
%   IndividualRow = NewStrokeRows(xx);   %give individual NewStrokeRows elements during the loop
%   EndStrokeRow = int32(IndividualRow) - 1;   %must convert to int32 to use in matrix assign value later
%  if EndStrokeRow > 1   %only if end stroke is later than first line then we edit it (because there is no line 0)
%   XYZmatrix(EndStrokeRow,3) = ZOffset + LiftPenHeight;
%  end
% end
% 
% 
% % %% For yaw (w) roll (P) and pitch (R)
% % %optional
% % 
% % WPRMatrix(1:XYZmatrixRowSize,1) = Yaw;   %make a number of rows same as XYZmatrix and first column equals yaw
% % WPRMatrix(1:XYZmatrixRowSize,2) = Roll;  %second column equals pitch
% % WPRMatrix(1:XYZmatrixRowSize,3) = Pitch;  %third column equals Pitch
% % 
% % XYZWPRMatrix = [XYZmatrix WPRMatrix];  %combine x,y,z and yaw pitch and roll to one sum matrix
% %% output to csv file
% 
% % writematrix(XYZWPRMatrix,'test.csv');     %write matrix to test.csv file 
% 
% writematrix(XYZmatrix, 'test.csv');
% 
% %Iinspired by Kevin Li (CC BY-SA 2.0)



