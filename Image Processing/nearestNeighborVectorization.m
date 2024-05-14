%% Nearest Neighbor Vectorization Function
function vectors = nearestNeighborVectorization(BW)
    [rows, cols] = size(BW);
    vectors = [];  % Initialize the vector array
    
    for r = 1:rows
        c = 1;
        while c <= cols
            if BW(r, c) == 1  % Edge pixel found
                start_col = c;
                
                % Find end of this segment in the row
                while c <= cols && BW(r, c) == 1
                    c = c + 1;
                end
                end_col = c - 1;
                
                % Store the vector as [start_row, start_col, end_row, end_col]
                vectors = [vectors; r, start_col, r, end_col];
            else
                c = c + 1;
            end
        end
    end
    
    % Repeat for vertical segments
    for c = 1:cols
        r = 1;
        while r <= rows
            if BW(r, c) == 1
                start_row = r;
                
                % Find end of this vertical segment
                while r <= rows && BW(r, c) == 1
                    r = r + 1;
                end
                end_row = r - 1;
                
                % Store the vector as [start_row, start_col, end_row, end_col]
                vectors = [vectors; start_row, c, end_row, c];
            else
                r = r + 1;
            end
        end
    end
    
    return
end