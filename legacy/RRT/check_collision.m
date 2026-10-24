% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q -> 1x4 vector denoting the configuration to check for collision
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: in_collision -> Boolean, true if the robot at configuration q is
%                         in collision with the given spherical obstacles

function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, cuboid_origin, cuboid_ckg, resolution)
    x1 = [0 0 0]';%第一个关节

    T2 = robot.A(1,q) ;
    x2 = T2.t;%第二个关节位置

    T3 = robot.A(1:2,q);
    x3 = T3.t;%第三个关节位置

    T4 = robot.A(1:3,q);
    x4 = T4.t;%第4个关节位置
    
    T5 = robot.A(1:4,q);
    x5 = T5.t;%第5个关节位置

    T6 = robot.A(1:5,q);
    x6 = T6.t;%第6个关节位置

    T7 = robot.A(1:6,q);
    x7 = T7.t;%末端位置


    if nargin < 8
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1); %在1和2两个关节之间取11个点，判断杆是否发生碰撞
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    x34 = repmat(x3, 1, n) + repmat(x4 - x3, 1, n) .* repmat(ticks, 3, 1);
    x45 = repmat(x4, 1, n) + repmat(x5 - x4, 1, n) .* repmat(ticks, 3, 1);
    x56 = repmat(x5, 1, n) + repmat(x6 - x5, 1, n) .* repmat(ticks, 3, 1);
    x67 = repmat(x6, 1, n) + repmat(x7 - x6, 1, n) .* repmat(ticks, 3, 1);
    points = [x12 x23 x34 x45 x56 x67];
    
    in_collision = false;
    for i = 1:size(sphere_centers, 1)
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break;
        end
    end
    if(~in_collision)
        for i = 1:size(cuboid_origin, 1)
            if any( points(1,:) > repmat(cuboid_origin(i,1)                  ,1,size(points,2)) & ...
                    points(1,:) < repmat(cuboid_origin(i,1) + cuboid_ckg(i,1),1,size(points,2)) & ...
                    points(2,:) > repmat(cuboid_origin(i,2)                  ,1,size(points,2)) & ...
                    points(2,:) < repmat(cuboid_origin(i,2) + cuboid_ckg(i,2),1,size(points,2)) & ...
                    points(3,:) > repmat(cuboid_origin(i,3)                  ,1,size(points,2)) & ...
                    points(3,:) < repmat(cuboid_origin(i,3) + cuboid_ckg(i,3),1,size(points,2))    )
                in_collision = true;
                break;
            end
        end    
    end
end