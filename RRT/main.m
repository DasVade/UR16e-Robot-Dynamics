%%% UR16e�����˶�ѧ�Ƶ�
clc;clear;close all
% ���ýǶȵ�λת��
degtorad = pi/180;

% ��������ƫ��
d1 = 180.7;
d2 = 0;
d3 = 0;
d4 = 174.15;
d5 = 119.85;
d6 = 116.55;
d = [d1,d2,d3,d4,d5,d6];

%�������˳���
a1 = 0;
a2 = -478.4;;
a3 = -360;
a4 = 0;
a5 = 0;
a6 = 0;
a = [a1,a2,a3,a4,a5,a6];

%��������Ť�ؽ�
alpha1 =   90 * degtorad;
alpha2 =   0 * degtorad;
alpha3 =   0 * degtorad;
alpha4 =   90 * degtorad;
alpha5 = -90 * degtorad;
alpha6 =  0 * degtorad;
alpha = [alpha1, alpha2,alpha3,alpha4,alpha5,alpha6];

L(1)=Link([ 0   d1   a1   alpha1], 'standard');
L(2)=Link([ 0   d2   a2   alpha2], 'standard');
L(3)=Link([ 0   d3   a3   alpha3], 'standard');
L(4)=Link([ 0   d4   a4   alpha4], 'standard');
L(5)=Link([ 0   d5   a5   alpha5], 'standard');
L(6)=Link([ 0   d6   a6   alpha6], 'standard');

robot=SerialLink(L,'name','robot');
figure(1)

view(3);
robot.plot([0,0,0,0,0,0],'tilesize',400)
robot.teach()
hold on

%% ��֤�����˶�ѧ
%�ؽ�5��0������ؽ�6�޷����
%q1 = [1,1,3.11,1,-1,-1.44];
% 
%T1 = robot.fkine(q1);
%T2 = myfkine(q1,a,d,alpha);
%p = T1.transl();
% 
%theta = myikine(T2,a,d);
%for i = 1:8
%    T = robot.fkine(theta(i,:),a,d,alpha);
%    pos(i,:) = T.transl();
%end



%% �켣�滮
% Start and goal configuration
q_start = [-0.276   -0.833   1.499   -2.255   -1.499   0];
q_goal  = [2.855   -0.833   1.499   -2.255   -1.499   0];
T_start = zeros(4, 4);
T_goal = zeros(4, 4);
T_start(:, :) = robot.fkine(q_start);
T_goal(:, :) = robot.fkine(q_goal);
figure(1)
plot3(T_goal(1, 4), T_goal(2, 4), T_goal(3, 4),'-o','Color','b','MarkerSize',10, 'MarkerFaceColor','b')
hold on
plot3(T_start(1, 4), T_start(2, 4), T_start(3, 4),'-o','Color','r','MarkerSize',10, 'MarkerFaceColor','b')
hold on

% Minimum and maximum joint angles for each joint
q_min = [ -pi -pi -pi/2 -pi/2 -pi/3 0];
q_max = [ pi  0  pi/2  pi/2  -pi/20 0];
% Radius of each robot link's cylindrical body
link_radius = 40;

% Set up spherical obstacle
sphere_center = [-300  0 150; 
                    300 0 150 ;
                    0  300 150];
sphere_radius = [150,150,150];

cuboid_origin = 4*[
                 -500,-500,-100;];
cuboid_ckg = 4*[
              1000,1000,100;];


draw_sphere(sphere_center,sphere_radius,3);% the third input is the num of sphere.
draw_cuboid(cuboid_origin,cuboid_ckg,1);

xlim([-1200,1200]);
ylim([-1200,1200]);
zlim([-1200,1200]);

%% Use the RRT algorithm 
% In order to ensure the efficiency of repetition, the path is stored here. 
% If you want to get a new path, you need to uncomment next three lines and comment next 4-5 lines.
 %[path, path_found] = RRT(robot, q_min, q_max, q_start, q_goal, link_radius, ...
    %sphere_center, sphere_radius, cuboid_origin, cuboid_ckg);
 %save('path.mat', 'path');
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


    %% b-spline 
    k = 3; % Using Cubic B-spline
    Pr = 50; % Precision of curve
    t = linspace(0,1,Pr);
    t_total = linspace(0,length(path.pos)+4-k,50*(length(path.pos)+4-k));
    theta = [];
    for i = 1:length(path.pos)
        theta(:,i) = path.pos(i).q;
    end
    if theta(1,end-1) < 0
        theta(1,end) = -pi;
    end
    theta = [theta(:,1) theta(:,1) theta theta(:,end) theta(:,end)];  % ��β���Ƶ��ظ���ʹ��������
    [q qd   qdd] = b_spline(theta,t,k); 
    
    T1 = zeros(4, 4, length(q));
    for i = 1:length(q)
        T1(:, :, i) = robot.fkine(q(:,i));
    end
    plot3(squeeze(T1(1, 4, :)), squeeze(T1(2, 4, :)), squeeze(T1(3, 4, :)), 'g-', 'LineWidth', 2);
    hold on
    q = q';
    qd = qd';
    qdd = qdd';
    robot.plot(q, 'fps', 5000);
    N = size(q, 1);                % ���ݵ���
    T_total = 20;                  % ��ʱ�䣨�룩��������Ĺ켣�趨
    t = linspace(0, T_total, N)';  % ����ʱ������ Nx1
    q= [t, q];
% ��ȡ����
q1(:,1) = q(:, 2);
q1= [t, q1];
q2(:,1) = q(:, 3);
q2= [t, q2];
q3(:,1) = q(:, 4);
q3= [t, q3];
q4(:,1) = q(:, 5);
q4= [t, q4];
q5(:,1) = q(:, 6);
q5= [t, q5];
q6(:,1) = q(:, 7);
q6= [t, q6];

%% ����Ƕȡ����ٶȡ��Ǽ��ٶ�
    figure(2)
    subplot(2,2,1)
    plot(t_total,q(:,1:6)/degtorad);
    title("�ؽڽǶ�");
    legend('q1','q2','q3','q4','q5','q6');
    subplot(2,2,2)
    plot(t_total,qd(:,1:6));
    title("�ؽڽ��ٶ�");
    legend('q1','q2','q3','q4','q5','q6');
    subplot(2,2,3)
    plot(t_total,qdd(:,1:6));
    title("�ؽڽǼ��ٶ�");
    legend('q1','q2','q3','q4','q5','q6');
    
else
    disp('No path found.');
end







