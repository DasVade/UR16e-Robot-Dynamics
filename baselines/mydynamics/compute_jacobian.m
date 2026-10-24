function J_i = compute_jacobian(q, link_num)
    dh_params = [
        0, 0.1807, pi/2;
        -0.4784, 0, 0;
        -0.36, 0, 0;
        0, 0.17415, pi/2;
        0, 0.11985, -pi/2;
        0, 0.11655, 0;
    ];
    com_data = cat(3, ...
        [0.000; -0.016; 0.030], ... % Link 1
        [0.302;  0.000; 0.160], ... % Link 2
        [0.194;  0.000; 0.065], ... % Link 3
        [0.000; -0.009; 0.011], ... % Link 4
        [0.000;  0.018; 0.012], ... % Link 5
        [0.000;  0.000; -0.044] ... % Link 6
    );
 
    num_joints = 6;
    T = zeros(4,4,num_joints);
    z = zeros(3,num_joints);
    o = zeros(3,num_joints);
    T_current = eye(4);
    
    % 计算各连杆的齐次变换矩阵
    for i = 1:num_joints
        a = dh_params(i,1);
        d = dh_params(i,2);
        alpha = dh_params(i,3);
        theta = q(i);
        
        Ti = dh_transform(theta, d, a, alpha);
        T_current = T_current*Ti ;
        T(:,:,i) = T_current;
        o(:,i+1) = T_current(1:3,4);
        z(:,i+1) = T_current(1:3,3);
    end
    
    % 计算指定连杆质心的世界坐标（修正点1）
    com_local = com_data(:,:,link_num);
    com_homogeneous = T(:,:,link_num) * [com_local; 1]; 
    com_world = com_homogeneous(1:3);
    o(:,1) =[0;0;0];
    z(:,1)=[0;0;1];
    % 构造雅可比矩阵
    J_i = zeros(6, num_joints);
    for i = 1:link_num
        Jv = cross(z(:,i), com_world - o(:,i)); 
        Jw = z(:,i);
        
        J_i(:,i) = [Jv; Jw];
    end
end