%%% UR5正逆运动学推导
clc;clear;close all
% 设置角度单位转换
degtorad = pi/180;

% 设置连杆偏距
d1 = 89.2;
d2 = 0;
d3 = 0;
d4 = -109.3;
d5 = 94.75;
d6 = 82.5;
d = [d1,d2,d3,d4,d5,d6];

%设置连杆长度
a1 = 0;
a2 = 0;
a3 = 425;
a4 = 392;
a5 = 0;
a6 = 0;
a = [a1,a2,a3,a4,a5,a6];

%设置连杆扭矩角
alpha1 =   0 * degtorad;
alpha2 = -90 * degtorad;
alpha3 =   0 * degtorad;
alpha4 =   0 * degtorad;
alpha5 =  90 * degtorad;
alpha6 =  90 * degtorad;
alpha = [alpha1, alpha2,alpha3,alpha4,alpha5,alpha6];

L(1)=Link([ 0   d1   a1   alpha1], 'modified');
L(2)=Link([ 0   d2   a2   alpha2], 'modified');L(2).offset = -pi/2; 
L(3)=Link([ 0   d3   a3   alpha3], 'modified'); 
L(4)=Link([ 0   d4   a4   alpha4], 'modified');L(4).offset = pi/2; 
L(5)=Link([ 0   d5   a5   alpha5], 'modified');
L(6)=Link([ 0   d6   a6   alpha6], 'modified');

robot=SerialLink(L,'name','robot');
figure(1)
view(3);
robot.plot([0,0,0,0,0,0],'tilesize',400)
robot.teach()
hold on

%% 验证正逆运动学
% 关节5≠0，否则关节6无法求解
% q1 = [1,1,3.11,1,-1,-1.44];
% 
% T1 = robot.fkine(q1);
% T2 = myfkine(q1,a,d,alpha);
% p = T1.transl();
% 
% theta = myikine(T2,a,d);
% for i = 1:8
%     T = robot.fkine(theta(i,:),a,d,alpha);
%     pos(i,:) = T.transl();
% end



%% 轨迹规划
% Start and goal configuration
q_start = [0   -pi/3   pi/6   pi/6   -pi/6   0];
q_goal  = [pi  -5*pi/6   pi/3       0 -pi/6 0];
T_start = zeros(4, 4);
T_goal = zeros(4, 4);
T_start(:, :) = robot.fkine(q_start);
T_goal(:, :) = robot.fkine(q_goal);
figure(1)
plot3(T_goal(1, 4), T_goal(2, 4), T_goal(3, 4),'-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b')
hold on
plot3(T_start(1, 4), T_start(2, 4), T_start(3, 4),'-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b')
hold on

% Minimum and maximum joint angles for each joint
q_min = [-pi -pi -pi/2 -pi/2 -pi/6 0];
q_max = [ pi  0  pi/2  pi/2  -pi/6 0];
% Radius of each robot link's cylindrical body
link_radius = 50;

% Set up spherical obstacle
sphere_center = [-400  0 -200; 
                    0 -600 80;
                    0  600 800;
                    0 0  500];
sphere_radius = [200,240,320,250];

cuboid_origin = 4*[
                 0,100,0;
                 100,-100,0;
                 130,-70,150];
cuboid_ckg = 4*[
              80,100,100;
              80,20,100;
              100,150,150];


draw_sphere(sphere_center,sphere_radius,4);% the third input is the num of sphere.
draw_cuboid(cuboid_origin,cuboid_ckg,3);

xlim([-1200,1200]);
ylim([-1200,1200]);
zlim([-1200,1200]);

%% Use the RRT algorithm 
% In order to ensure the efficiency of repetition, the path is stored here. 
% If you want to get a new path, you need to uncomment next three lines and comment next 4-5 lines.
% [path, path_found] = RRT(robot, q_min, q_max, q_start, q_goal, link_radius, ...
%     sphere_center, sphere_radius, cuboid_origin, cuboid_ckg);
% save('path.mat', 'path');
load('path.mat');
path_found = true;

% Visualize the trajectory, if one is found
if path_found
    % Plot the trajectory found.
    path.pos = flip(path.pos,2);
    T = zeros(4, 4, length(path.pos));
    for i = 1:length(path.pos)
        T(:, :, i) = robot.fkine(path.pos(i).q);
    end
    plot3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)), 'r-', 'LineWidth', 2);
    hold on

%     %% smooth
%     % If trajectory is found, smooth the trajectory
%     smoothed_path = SmoothenPath(robot, path.pos, link_radius, sphere_center, sphere_radius, cuboid_origin, cuboid_ckg);
%     
%     % Visualize the smoothed trajectory
%     fprintf('Smoothed path found with %d intermediate waypoints:\n', size(smoothed_path, 1) - 2);
%     disp(smoothed_path);
% 
%     % Plot the trajectory smoothed.
%     T = zeros(4, 4, size(smoothed_path, 1));
%     for i = 1:size(smoothed_path, 1)
%         T(:, :, i) = robot.fkine(smoothed_path(i,:));
%     end
%     plot3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)), 'k-', 'LineWidth', 2);
%     hold on

    %% b-spline + 四元数插值
    k = 3; % Using Cubic B-spline
    Pr = 50; % Precision of curve
    t = linspace(0,1,Pr);
    t_total = linspace(0,50,2500);
    
    pos_path = [squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :))].';
    pos_path = [pos_path(:,1) pos_path(:,1) pos_path pos_path(:,end) pos_path(:,end)];  % 首尾控制点重复，使线条收敛
    [inter_path_b_spline,v,as] = b_spline(pos_path, t, k);                             % 对位置使用b_spline插值，路径序列翻转，起点放到前面
    plot3(inter_path_b_spline(1,:), inter_path_b_spline(2,:), inter_path_b_spline(3,:), ...
        'g-','LineWidth', 2, 'DisplayName', 'Cubic-B-spline');
    
    % 姿态插值
    R = T; 
    R(4,:,:) = [];
    R(:,4,:) = [];
    R = cat(3,R(:,:,1),R);
    R = cat(3,R,R(:,:,end));
    R = cat(3,R(:,:,1),R);
    R = cat(3,R,R(:,:,end));
    inter_R = [];
    R_cul = [];
    omega = [];
    omega_d = [];
    for i = 1:length(R)-3
        tmp = interp(UnitQuaternion(R(:,:,i+1)),UnitQuaternion(R(:,:,i+2)),t,'slerp');
        inter_R = cat(3,inter_R,quat2rotm(tmp.double));
    end
    R_cul = cat(3,R(:,:,1),inter_R);
    R_cul = cat(3,R_cul,R(:,:,end));
    q_cul = rotm2quat(R_cul);
    for i = 1:length(q_cul)-1
        q_cul_d = (q_cul(i+1,:) - q_cul(i,:))/0.02;
        q_cul_matrix = [q_cul(i,1) -q_cul(i,2) -q_cul(i,3) -q_cul(i,4);
                            q_cul(i,2) q_cul(i,1) -q_cul(i,4) q_cul(i,3);
                            q_cul(i,3) q_cul(i,4) q_cul(i,1) -q_cul(i,2);
                            q_cul(i,4) -q_cul(i,3) q_cul(i,2) q_cul(i,1)];
        omega = [omega;q_cul_d*2/q_cul_matrix];
    end       
    omega(:,1) = [];
    for i = 1:length(q_cul)-2
        omega_d = [omega_d;(omega(i+1,:) - omega(i,:))/0.02];
    end

    % 将旋转矩阵和位置合并成齐次变换矩阵
    inter_T_b_spline = zeros(4,4,length(inter_R));
    for i = 1:length(inter_R)
        inter_T_b_spline(1:3,1:4,i) = [inter_R(:,:,i) inter_path_b_spline(:,i)];
        inter_T_b_spline(4,:,i) = [0 0 0 1];
    end

    % 将T变换矩阵求逆解得到关节角度
    theta = zeros(8,6,length(inter_T_b_spline));
    for i = 1:length(inter_T_b_spline)
        theta(:,:,i)= myikine(inter_T_b_spline(:,:,i),a,d);
    end
   
    theta_choose = zeros(size(theta,3),6);
    theta_choose(1,:) = q_start;
    for i = 2:size(theta,3)+1
        if i == 649 % 断点，不必理会这个if从句
            i = i;
        end
        e = 1000000; % 用来选择8组解中离上一个theta最近的解    
        for j = 1:8
            if norm(theta(j,:,i-1)-theta_choose(i-1,:))<e
                e = norm(theta(j,:,i-1)-theta_choose(i-1,:));
                theta_choose(i,:) = theta(j,:,i-1);
                theta(j,:,i-1)
                e
            end
        end
    end
    theta_choose(1,:) = [];
%     robot.plot(theta_choose, 'fps', 5000);

%% 输出角度、角速度、角加速度
    omega(end,:) = [];
    V = [v;omega'];
    A = [as;omega_d'];
    for i = length(theta_choose)
        J(:,:,i) = robot.jacob0(theta_choose(i,:));
        thetad = inv(J(:,:,i))*V;
        thetadd = inv(J(:,:,i))*A;
       
    end
    figure(2)
    subplot(2,1,1)
    plot(t_total,v(1:3,:)'/degtorad);
    title("末端速度");
    legend('x','y','z');

    subplot(2,1,2)
    plot(t_total,as(1:3,:)'/degtorad);
    title("末端加速度");
    legend('x','y','z');

    figure(3)
    subplot(2,2,1)
    plot(t_total,theta_choose(:,1:6)'/degtorad);
    title("关节角度");
    legend('q1','q2','q3','q4','q5','q6');
    subplot(2,2,2)
    plot(t_total,thetad(1:6,:));
    title("关节角速度");
    legend('q1','q2','q3','q4','q5','q6');
    subplot(2,2,3)
    plot(t_total,thetadd(1:6,:));
    title("关节角加速度");
    legend('q1','q2','q3','q4','q5','q6');
    
else
    disp('No path found.');
end







