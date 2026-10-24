% RRT algorithm

function [path, path_found] = RRT(robot, q_min, q_max, q_start, q_goal, link_radius, ...
    sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
    % Define the configuration space
    q_limits = [q_min;q_max];
    
    % Initialize the tree with the start configuration as the root node
    t.v(1).q = q_start;
    t.v(1).q_pre = q_start;
    t.v(1).dist = 0;
    t.v(1).idx_pre = 0;
    
    % Define the maximum number of iterations
    max_iterations = 100000;
    
    % Define the step size for extending the tree
    step_size = 0.8;
    
    % Define the goal region   !! If more obstacles are added near the goal
    % point, this value needs to be reduced.
    goal_region = 50;
    
    % Initialize the flag for reaching the goal configuration
    path_found = false;
    count = 1;
    
    m=0.5;
    for i = 1:max_iterations
        % Generate a random configuration
        if rand>m
            q_rand = q_limits(1,:) + (q_limits(2,:) - q_limits(1,:)).*rand(1,6);
            robot.plot(q_rand);
        else
            q_rand=q_goal;
        end
        % Find the nearest node in the tree
        dist = arrayfun((@(x) vecnorm(x.q - q_rand, 2, 2)),t.v);
        [min_dist, idx] = min(dist);
        q_near = t.v(idx).q;
        
        % Extend the tree towards the random configuration
        delta_q = step_size * (q_rand - q_near) / min_dist;
        q_new = q_near + delta_q;
        robot.plot(q_new);
        
    
        % Check if the new configuration is valid and collision-free
        if ~check_edge(robot, q_near, q_new, link_radius, sphere_center, sphere_radius, cuboid_origin, cuboid_ckg)
            % Add the new configuration to the tree as a new node
            count = count +1;
            t.v(count).q = q_new;
            t.v(count).q_pre = q_near;
            t.v(count).dist = delta_q;
            t.v(count).idx_pre = idx;
            
            T = zeros(4, 4, 2);
            T(:, :, 2) = robot.fkine(q_new);
            T(:, :, 1) = robot.fkine(q_near);
            plot3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)), 'k-', 'LineWidth', 1);
            hold on


            T(:,:,3) = robot.fkine(q_goal);
            % Check if the goal configuration is reached
            if norm([T(1, 4, 2), T(2, 4, 2), T(3, 4, 2)] - [T(1, 4, 3), T(2, 4, 3), T(3, 4, 3)]) <= goal_region
                fprintf('the size of tree is: %d\n', count);
                path_found = true;
                break;
            else
                % 使用 sprintf 组合字符串和数值
                str = sprintf('the size of tree is: %d the current distance is: %f', count, norm([T(1, 4, 2), T(2, 4, 2), T(3, 4, 2)] - [T(1, 4, 3), T(2, 4, 3), T(3, 4, 3)]));
                % 把组合好的字符串传递给 disp 函数
               disp(str);
            end
        end
    end
    
    if path_found 
        path.pos(1).q = q_goal; 
        path.pos(2).q = t.v(end).q; 
        pathIndex = t.v(end).idx_pre; % 终点加入路径
        j=0;
        while 1
            path.pos(j+3).q = t.v(pathIndex).q;
            pathIndex = t.v(pathIndex).idx_pre;
            fprintf('back to: %d\n', pathIndex);
            if pathIndex == 1
                break
            end
            j=j+1;
        end  % 沿终点回溯到起点
        path.pos(end+1).q = q_start;  % 起点加入路径
        
    else
        path = 0;
    end
end