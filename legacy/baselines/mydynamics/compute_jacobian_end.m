function J_end = compute_jacobian_end(q)
    dh_params = [
        0, 0.1807, pi/2;
        -0.4784, 0, 0;
        -0.36, 0, 0;
        0, 0.17415, pi/2;
        0, 0.11985, -pi/2;
        0, 0.11655, 0;
    ];
    
    num_joints = 6;
    T = zeros(4,4,num_joints);
    z = zeros(3,num_joints);
    o = zeros(3,num_joints);
    T_current = eye(4);
    
    for i = 1:num_joints
        a = dh_params(i,1);
        d = dh_params(i,2);
        alpha = dh_params(i,3);
        theta = q(i);
        Ti = dh_transform(theta, d, a, alpha);
        T_current = T_current*Ti;
        T(:,:,i) = T_current;
        o(:,i+1) = T_current(1:3,4);
        z(:,i+1) = T_current(1:3,3);
    end
    o(:,1) =[0;0;0];
    z(:,1)=[0;0;1];
    % 末端执行器位置（Tool坐标系原点）
    end_effector_pos = o(:,end);
    
    J_end = zeros(6, num_joints);
    for i = 1:num_joints
        Jv = cross(z(:,i), end_effector_pos - o(:,i));
        Jw = z(:,i);
        J_end(:,i) = [Jv; Jw];
    end
end
