function path = astar(maze, start, goal)
    directions = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
    rows = size(maze, 1);
    cols = size(maze, 2);
    
    heuristic = @(a, b) sqrt((a(1) - b(1))^2 + (a(2) - b(2))^2);
    
    openSet = start;
    gScore = inf(rows, cols);
    gScore(start(1), start(2)) = 0;
    fScore = inf(rows, cols);
    fScore(start(1), start(2)) = heuristic(start, goal);
    cameFrom = zeros(rows, cols, 2);

    while ~isempty(openSet)
        [~, idx] = min(fScore(sub2ind([rows, cols], openSet(:,1), openSet(:,2))));
        current = openSet(idx, :);
        
        if isequal(current, goal)
            path = reconstruct_path(cameFrom, current);
            %path = path(:, [2, 1]);
            return;
        end
        
        openSet(idx, :) = [];
        fScore(current(1), current(2)) = inf;

        for i = 1:size(directions, 1)
            neighbour = current + directions(i, :);
            
            if is_valid_neighbour(neighbour, maze)
                tentative_gScore = gScore(current(1), current(2)) + 1;
                
                if tentative_gScore < gScore(neighbour(1), neighbour(2))
                    cameFrom(neighbour(1), neighbour(2), :) = current;
                    gScore(neighbour(1), neighbour(2)) = tentative_gScore;
                    fScore(neighbour(1), neighbour(2)) = tentative_gScore + heuristic(neighbour, goal);
                    
                    if ~ismember(neighbour, openSet, 'rows')
                        openSet = [openSet; neighbour];
                    end
                end
            end
        end
    end
    path = [];
end

function valid = is_valid_neighbour(neighbour, maze)
    rows = size(maze, 1);
    cols = size(maze, 2);
    valid = neighbour(1) > 0 && neighbour(1) <= rows && neighbour(2) > 0 && neighbour(2) <= cols && maze(neighbour(1), neighbour(2)) == 0;
end

function path = reconstruct_path(cameFrom, current)
    path = [];
    while any(current ~= 0)
        path = [current; path];
        current = squeeze(cameFrom(current(1), current(2), :))';
    end
end
