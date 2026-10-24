function tau = robot_inversedynamics(q,q_dot,q_accel,F_end)
% Same external-wrench convention as robot_forwarddynamics.
[m,I]=urdf_parameters();M=compute_mass_matrix(q,m,I);
C=compute_coriolis(q,q_dot,m,I);G=compute_gravity(q,m,9.81);J=compute_jacobian_end(q);
tau=M*q_accel(:)+C*q_dot(:)+G-J'*F_end(:);
end
