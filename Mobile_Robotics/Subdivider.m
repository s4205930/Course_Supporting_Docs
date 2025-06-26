clc; clear; close all;

% Load the maze from the .mat file
% Assume the .mat file contains a matrix 'maze'
load('maze.mat', 'maze', 'start', 'goal');

% Parameters for subdividing the maze
subdivide_factor = 4;  % Number of subdivisions per original cell (e.g., 2x2 grid cells per original cell)

% Get the original maze size
[original_rows, original_cols] = size(maze);

% New grid dimensions after subdivision
new_rows = original_rows * subdivide_factor;
new_cols = original_cols * subdivide_factor;

% Create a new maze with higher resolution (initially all open space)
new_maze = zeros(new_rows, new_cols);

% Loop over the original maze and subdivide cells
for r = 1:original_rows
    for c = 1:original_cols
        % Check the current cell in the original maze
        if maze(r, c) == 1  % Wall
            % Mark the corresponding subgrid as walls
            new_maze((r-1)*subdivide_factor + 1 : r*subdivide_factor, ...
                     (c-1)*subdivide_factor + 1 : c*subdivide_factor) = 1;
        else  % Open space
            % Leave the corresponding subgrid as open space
            new_maze((r-1)*subdivide_factor + 1 : r*subdivide_factor, ...
                     (c-1)*subdivide_factor + 1 : c*subdivide_factor) = 0;
        end
    end
end

maze = new_maze;

% Adjust the start and goal positions based on the subdivision factor
% The original start and goal are indices in the original maze
start = (start - 1) * subdivide_factor + subdivide_factor/2;
goal = (goal - 1) * subdivide_factor + subdivide_factor/2;

save('maze_sub', 'maze', 'start', 'goal')