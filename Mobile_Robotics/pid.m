function [v_left, v_right] = pid(target, robot_pos, dt)
    global prev_error integral Kp Ki Kd max_angular_velocity v_max wheel_r axel_len;

    if dt <= 0
        error('Time step dt must be greater than zero.');
    end

    % Calculate desired angle and error
    desired_angle = atan2(target(2) - robot_pos(2), target(1) - robot_pos(1));
    angle_diff = desired_angle - robot_pos(3);
    error = mod(angle_diff + pi, 2*pi) - pi;  % Normalize error to [-pi, pi]

    % Update integral and derivative terms
    integral = integral + error * dt;
    derivative = (error - prev_error) / dt;
    
    % Compute angular velocity (w) using PID control
    w = Kp * error + Ki * integral + Kd * derivative;

    % Saturate angular velocity
    w = max(min(w, max_angular_velocity), -max_angular_velocity);
    
    % Compute forward velocity based on angular velocity
    v = v_max * (1 - abs(w) / max_angular_velocity);  

    % Convert linear velocity and angular velocity into wheel velocities
    v_left = (v - (w * axel_len) / 2) / wheel_r;
    v_right = (v + (w * axel_len) / 2) / wheel_r;

    % Save the current error for the next iteration
    prev_error = error;
end
