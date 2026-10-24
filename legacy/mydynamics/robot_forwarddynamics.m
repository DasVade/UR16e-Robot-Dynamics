function q_accel = robot_forwarddynamics(q,q_dot,F_end,tau)
% F_end is wrench ON robot at frame-6 origin, expressed in UR base,
% ordered [Fx Fy Fz Mx My Mz]. M*ddq+C*dq+G=tau+J'*F_end.
[m,I]=urdf_parameters();M=compute_mass_matrix(q,m,I);
C=compute_coriolis(q,q_dot,m,I);G=compute_gravity(q,m,9.81);J=compute_jacobian_end(q);
q_accel=M\(tau(:)-C*q_dot(:)-G+J'*F_end(:));
end
