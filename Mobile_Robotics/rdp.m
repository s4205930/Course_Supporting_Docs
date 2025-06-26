function simplified_path = rdp(path, epsilon)
    simplified_path = rdp_recursive(path, 1, size(path, 1), epsilon);
end

function line = rdp_recursive(path, start_idx, end_idx, epsilon)
    if end_idx - start_idx <= 1
        line = path([start_idx, end_idx], :);
        return;
    end
    
    line_start = path(start_idx, :);
    line_end = path(end_idx, :);
    dists = point_to_line_distance(path(start_idx+1:end_idx-1, :), line_start, line_end);
    
    [max_dist, idx_max] = max(dists);
    
    if max_dist > epsilon
        middle_point = path(start_idx + idx_max, :);
        line1 = rdp_recursive(path, start_idx, start_idx + idx_max, epsilon);
        line2 = rdp_recursive(path, start_idx + idx_max, end_idx, epsilon);
        line = [line1; line2(2:end, :)];
    else
        line = [line_start; line_end];
    end
end

function dists = point_to_line_distance(points, line_start, line_end)
    % Calculate the perpendicular distance from each point to the line
    % defined by line_start and line_end
    v1 = line_end - line_start;
    v2 = points - line_start;
    area = abs(v1(1) * v2(:, 2) - v1(2) * v2(:, 1));
    length_v1 = sqrt(v1(1)^2 + v1(2)^2);
    dists = area / length_v1;
end
