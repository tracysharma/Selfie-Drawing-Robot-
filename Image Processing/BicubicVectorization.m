function vectors = BicubicVectorization(BW, method)
    if nargin < 2
        method = 'bicubic'; % Default to bicubic if no method is specified
    end
    
    [rows, cols] = size(BW);
    % Prepare a grid for interpolation
    [X, Y] = meshgrid(1:cols, 1:rows);
    % Create an interpolated grid
    newX = linspace(1, cols, cols*2); % Increase density for smoothness
    newY = linspace(1, rows, rows*2);
    [newXGrid, newYGrid] = meshgrid(newX, newY);
    
    % Interpolate based on the specified method
    if strcmp(method, 'bilinear')
        newBW = interp2(X, Y, double(BW), newXGrid, newYGrid, 'linear');
    elseif strcmp(method, 'bicubic')
        newBW = interp2(X, Y, double(BW), newXGrid, newYGrid, 'cubic');
    else
        error('Unsupported interpolation method. Choose either "bilinear" or "bicubic".');
    end
    
    % Perform edge detection on the interpolated image
    newBWEdges = edge(newBW > 0.5, 'canny');
    
    % Extract the edge points and convert them into the format [x, y]
    [edgeY, edgeX] = find(newBWEdges);
    vectors = [edgeX, edgeY];  % Ensure the output is in [x, y] format
end