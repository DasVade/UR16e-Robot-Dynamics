function G = compute_gravity(q, masses, g)
    % 输入：
    % q: 关节角度 [6x1]
    % masses: 各连杆质量 [6x1]
    % g: 重力加速度标量（单位：m/s²，向下为负）
    
    num_joints = 6;
    G = zeros(num_joints, 1);
    gravity_vec = [0; 0; -g];  % 重力方向为基座坐标系的 -z
    
    for i = 1:num_joints
        % 计算连杆i质心的线速度雅可比矩阵（3x6）
        J_i = compute_jacobian(q, i);
        Jv_i=J_i(1:3,1:6);
        % 累加重力项：G += mass_i * Jv_i^T * gravity_vec
        G = G + masses(i) * Jv_i' * gravity_vec;
    end
end