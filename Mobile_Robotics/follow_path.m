function follow_path(path, maze, start, goal)
    % Define global variables for PID control and robot parameters
    global prev_error integral Kp Ki Kd max_angular_velocity v_max axel_len wheel_r;
    prev_error = 0;
    integral = 0;
    
    % PID control gains
    Kp = 1.75;
    Ki = 0.1;
    Kd = 0.01;
    
    % Robot movement constraints
    max_angular_velocity = 4;
    v_max = 2;
    axel_len = 0.5;
    wheel_r = 0.2;
    
    % Time step initialization
    time = 0;
    dt = 0.1;
    
    % Initialize robot position (x, y, theta)
    robot_pos = [start(1); start(2); 0];
    
    % Initialize odometry and EKF estimates
    odom_est = robot_pos;
    ekf_est = robot_pos;
    ekf_cov = eye(3) * 0.1; % Initial covariance matrix
    
    % Process noise covariance matrix
    Q = diag([0.05, 0.05, 0.05]);
    
    % Odometry noise standard deviations
    odm_noise = [0.8; 0.8; 0.5];
    R_odom = diag(odm_noise .^ 2);
    
    % Set up figure window size and position
    screenSize = get(0, 'ScreenSize');
    figWidth = 800;
    figHeight = 800;
    xPos = (screenSize(3) - figWidth) / 2;
    yPos = (screenSize(4) - figHeight) / 2;
    
    % Create figure for visualization
    figure('Name', 'Differential Drive Robot Simulation', 'Position', [xPos, yPos, figWidth, figHeight]);
    
    % Plot maze, start, goal, and path
    subplot(2, 2, 1);
    imagesc(maze);
    set(gca, 'YDir', 'normal');
    hold on;
    colormap([1 1 1; 0 0 0]); % White for free space, black for obstacles
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    if ~isempty(path)
        plot(path(:, 2), path(:, 1), 'r', 'LineWidth', 2);
    else
        disp('No path found!');
    end
    axis equal;
    title('Maze with Start, Goal, and Path');
    
    % Subplot for wheel speeds
    subplot(2, 2, 2);
    hold on;
    left_speed_line = plot(nan, nan, 'b', 'LineWidth', 2, 'DisplayName', 'Left Wheel Speed');
    right_speed_line = plot(nan, nan, 'r', 'LineWidth', 2, 'DisplayName', 'Right Wheel Speed');
    xlabel('Time (s)')
    ylabel('Wheel Speed')

    legend([left_speed_line, right_speed_line], ...
       {'Left Wheel Speed', 'Right Wheel Speed'}, ...
       'Location', 'southoutside');

    % Subplot for EKF error
    subplot(2, 2, 3);
    hold on;
    ekf_error_line = plot(nan, nan, 'g', 'LineWidth', 2, 'DisplayName', 'EKF Error');
    odom_error_line = plot(nan, nan, 'm', 'LineWidth', 2, 'DisplayName', 'Odometry Error');
    xlabel('Time (s)')
    ylabel('Euclidean Error (m)')

    legend([ekf_error_line, odom_error_line], ...
       {'EKF Error', 'Odometry Error'}, ...
       'Location', 'southoutside');
    
    % Subplot for position estimates
    subplot(2, 2, 1);
    odom_est_line = plot(nan, nan, 'm', 'LineWidth', 2, 'DisplayName', 'Odometry Estimate');
    ekf_est_line = plot(nan, nan, 'g', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');
    gps_true_line = plot(nan, nan, 'b', 'LineWidth', 2, 'DisplayName', 'GPS True');

    legend([odom_est_line, ekf_est_line, gps_true_line], ...
       {'Odometry Estimate', 'EKF Estimate', 'GPS True'}, ...
       'Location', 'southoutside');
    
    
    % Follow the path step by step
    for step = 1:size(path, 1)
        target = path(step, [2, 1]);
        
        while norm(robot_pos(1:2)' - target) > 0.4  
            
            % Compute wheel velocities using PID controller
            [v_left, v_right] = pid(target, robot_pos, dt);
            
            % Compute robot motion using forward kinematics
            [v, w] = forwardKinematics(wheel_r, axel_len, v_left, v_right);
            local_v = [v; 0; w];
            global_v = local_to_global(local_v, robot_pos);
            
            % Update robot position
            robot_pos = robot_pos + global_v * dt;
            
            % Odometry prediction update
            dT = w * dt;
            dX = v * cos(ekf_est(3)) * dt;
            dY = v * sin(ekf_est(3)) * dt;
            odom_est = odom_est + [dX; dY; dT];
            
            % EKF prediction step
            ekf_predicted = ekf_est + [dX; dY; dT];
            F = eye(3);
            F(1,3) = -v * sin(ekf_est(3)) * dt;
            F(2,3) = v * cos(ekf_est(3)) * dt;
            ekfCov_predicted = F * ekf_cov * F' + Q;
            
            % EKF update using odometry
            H_odom = eye(3);
            z_odom = odom_est;
            zPred_odom = H_odom * ekf_predicted;
            y_odom = z_odom - zPred_odom;
            S_odom = H_odom * ekfCov_predicted * H_odom' + R_odom;
            K_odom = ekfCov_predicted * H_odom' / S_odom;
            ekf_est = ekf_predicted + K_odom * y_odom;
            ekf_cov = (eye(3) - K_odom * H_odom) * ekfCov_predicted;
            
            % GPS measurement update
            gps_est = simulate_gps(robot_pos(1:2), 0.5);
            H_gps = [1, 0, 0; 0, 1, 0];
            z_gps = gps_est;
            zPred_gps = H_gps * ekf_est;
            y_gps = z_gps - zPred_gps;
            S_gps = H_gps * ekf_cov * H_gps';
            K_gps = ekf_cov * H_gps' / S_gps;
            ekf_est = ekf_est + K_gps * y_gps;
            ekf_cov = (eye(3) - K_gps * H_gps) * ekf_cov;

            subplot(2, 2, 1);
            gps_true_line.XData = [gps_true_line.XData, robot_pos(1)];
            gps_true_line.YData = [gps_true_line.YData, robot_pos(2)];

            ekf_est_line.XData = [ekf_est_line.XData, ekf_est(1)];
            ekf_est_line.YData = [ekf_est_line.YData, ekf_est(2)];

            odom_est_line.XData = [odom_est_line.XData, odom_est(1)];
            odom_est_line.YData = [odom_est_line.YData, odom_est(2)];

            subplot(2, 2, 2);
            left_speed_line.XData = [left_speed_line.XData, time];
            left_speed_line.YData = [left_speed_line.YData, v_left];

            right_speed_line.XData = [right_speed_line.XData, time];
            right_speed_line.YData = [right_speed_line.YData, v_right];

            subplot(2, 2, 3);
            ekf_error_line.XData = [ekf_error_line.XData, time];
            ekf_error_line.YData = [ekf_error_line.YData, sqrt((robot_pos(1) - ekf_est(1))^2 + (robot_pos(2) - ekf_est(2))^2)];
            
            odom_error_line.XData = [odom_error_line.XData, time];
            odom_error_line.YData = [odom_error_line.YData, sqrt((robot_pos(1) - odom_est(1))^2 + (robot_pos(2) - odom_est(2))^2)];

            time = time + dt;
            pause(dt);
        end
    end
end

function gps_est = simulate_gps(true_position, noise_std)
    % Simulate GPS measurement with Gaussian noise
    gps_est = true_position + noise_std * randn(1, 2);
end

function [v, w] = forwardKinematics(wheel_r, axel_len, wL, wR)
    % Compute linear and angular velocities from wheel speeds
    v = 0.5 * wheel_r * (wL + wR);
    w = (wR - wL) * wheel_r / axel_len;
end

function global_v = local_to_global(local_v, robot_pos)
    % Convert local velocities to global frame
    direction = robot_pos(3);
    rotationMatrix = [cos(direction), -sin(direction), 0;
                      sin(direction),  cos(direction), 0;
                      0,             0,            1];
    global_v = rotationMatrix * local_v;
end