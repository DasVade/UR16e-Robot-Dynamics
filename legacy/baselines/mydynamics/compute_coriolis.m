function C = compute_coriolis(q, q_dot, masses, I_links)
    % 输入：
    % q: 关节角度 [6x1]
    % q_dot: 关节速度 [6x1]
    % masses: 各连杆质量 [6x1]
    % I_links: 各连杆在基座坐标系中的惯性张量 [3x3x6]
    
    num_joints = 6;
    C = zeros(num_joints);
    
    for i = 1:num_joints
        % 获取连杆i的线速度和角速度雅可比矩阵
        J_i = compute_jacobian(q, i);
        Jv_i=J_i(1:3,:);
        Jw_i=J_i(4:6,:);
        % 计算连杆i的线速度和角速度
        v_i = Jv_i * q_dot;          % 线速度 [3x1]
        omega_i = Jw_i * q_dot;      % 角速度 [3x1]
        
        % 计算科氏力和离心力项
        term_v = masses(i) * skew(omega_i) * v_i;               % 线速度相关项
        term_w = skew(omega_i) * I_links(:, :, i) * omega_i;    % 角速度相关项
        
        % 投影到关节空间并累加
        C = C + Jv_i' * term_v + Jw_i' * term_w;
    end
end