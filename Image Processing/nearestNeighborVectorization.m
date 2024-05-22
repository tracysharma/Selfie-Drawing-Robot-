%% Nearest Neighbor Vectorization Function
function vectors = nearestNeighborVectorization(BW)
    [rows, cols] = size(BW); % Determine the size of the binary image
    vectors = [];  % Initialize the vector array to store the start and end points of line segments
    
    % Process each row to find horizontal line segments
    for r = 1:rows
        c = 1; % Start column index at 1
        while c <= cols
            if BW(r, c) == 1  % Check if the current pixel is an edge pixel
                start_col = c; % Mark the start of a line segment
                
                % Continue until the end of this segment is found
                while c <= cols && BW(r, c) == 1
                    c = c + 1;
                end
                end_col = c - 1; % Mark the end of the line segment
                
                % Store the vector as [start_row, start_col, end_row, end_col]
                vectors = [vectors; r, start_col, r, end_col];
            else
                c = c + 1;  % Move to the next column
            end
        end
    end
    
    % Repeat for vertical segments
    for c = 1:cols
        r = 1;
        while r <= rows
            if BW(r, c) == 1 % Check if the current pixel is an edge pixel
                start_row = r; % Mark the start of a line segment
                
                % Find end of this vertical segment
                while r <= rows && BW(r, c) == 1
                    r = r + 1;
                end
                end_row = r - 1; % Mark the end of the line segment
                
                % Store the vector as [start_row, start_col, end_row, end_col]
                vectors = [vectors; start_row, c, end_row, c];
            else
                r = r + 1; % Move to the next row
            end
        end
    end
    
    return
end