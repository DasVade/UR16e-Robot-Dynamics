function M = compute_mass_matrix(q, masses, I_links)
    % 输入：
    % q: 关节角度[6x1]
    % masses: 各连杆质量向量 [6x1]
    % I_links: 各连杆在质心坐标系中的惯性张量 [3x3x6]
    num_joints = 6;
    M = zeros(num_joints);
    T = zeros(4,4,num_joints);
    z = zeros(3,num_joints);
    o = zeros(3,num_joints);
    T_current = eye(4);
    dh_params = [
        0, 0.1807, pi/2;
        -0.4784, 0, 0;
        -0.36, 0, 0;
        0, 0.17415, pi/2;
        0, 0.11985, -pi/2;
        0, 0.11655, 0;
    ];
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
    for i = 1:num_joints
        % 获取连杆i的线速度和角速度雅可比
        J_i = compute_jacobian(q, i);
        Jv_i=J_i(1:3,:);
        Jw_i=J_i(4:6,:);
        % 获取连杆i的旋转矩阵
        R_i=T(1:3,1:3,i);
        % 转换惯性张量到基座坐标系
        I_world = R_i * I_links(:,:,i) * R_i';
        
        % 累加质量矩阵
        M = M + Jv_i' * masses(i) * Jv_i + Jw_i' * I_world * Jw_i;
    end
end