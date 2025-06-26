clc; clear; close all;

%% Maze Parameters
rows = 13; % Maze height (must be odd for proper walls)
cols = 13; % Maze width (must be odd for proper walls)
maze = ones(rows, cols); % Initialize with walls
start = [2, 2]; % Start position
goal = [rows-1, cols-1]; % Goal position
visited = zeros(rows, cols); % Keep track of visited cells

%% Recursive DFS function with proper updating
function [maze, visited] = generate_maze(maze, visited, r, c, rows, cols)
    visited(r, c) = 1; % Mark as visited
    maze(r, c) = 0; % Open this cell (make it a path)

    % Move 2 steps in each direction (down, right, up, left)
    directions = [0 2; 2 0; 0 -2; -2 0];  
    dir_order = randperm(4); % Shuffle movement directions

    for i = dir_order
        new_r = r + directions(i, 1);
        new_c = c + directions(i, 2);

        % Check if new cell is inside bounds and not visited
        if new_r > 1 && new_r < rows && new_c > 1 && new_c < cols && visited(new_r, new_c) == 0
            % Remove wall between current and new cell
            maze((r + new_r) / 2, (c + new_c) / 2) = 0;
            
            % Recursive call and update maze and visited
            [maze, visited] = generate_maze(maze, visited, new_r, new_c, rows, cols);
        end
    end
end

%% Start Maze Generation
[maze, visited] = generate_maze(maze, visited, start(1), start(2), rows, cols);

%% Ensure Start & Goal Are Open
maze(start(1), start(2)) = 0;
maze(goal(1), goal(2)) = 0;

%% Plot Maze
figure;
imagesc(maze);
hold on;
colormap([1 1 1; 0 0 0]); % White = path (0), Black = wall (1)

plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start (green)
plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Goal (red)
title('Generated Maze');
axis equal off;

%% Save Maze
save('maze.mat', 'maze', 'start', 'goal');