clc; clear; close all;

%Generate Enviroment
run Maze_Generator.m
run Subdivider.m

% Load Maze Data
load('maze_sub.mat', 'maze', 'start', 'goal'); 

% Define a safety margin
safety_margin = 1;  % Increase for more wall clearance

% Create a structuring element (square) for dilation
se = strel('square', 2 * safety_margin + 1);

% Inflate the walls (expand obstacles)
inflated_maze = imdilate(maze, se);

% Find path using A*
path = astar(inflated_maze, start, goal);
path = rdp(path, 1); % Simplify path

% Run simulation loop
follow_path(path, maze, start, goal);