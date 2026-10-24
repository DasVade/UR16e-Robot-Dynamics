function C = compute_coriolis(q,q_dot,masses,I_links)
% Christoffel C matrix; C*q_dot is a 6x1 torque. Central differences of M.
q=q(:);q_dot=q_dot(:);h=2e-5;dM=zeros(6,6,6);C=zeros(6);
for k=1:6
    e=zeros(6,1);e(k)=h;
    dM(:,:,k)=(compute_mass_matrix(q+e,masses,I_links)-compute_mass_matrix(q-e,masses,I_links))/(2*h);
end
for i=1:6
    for j=1:6
        for k=1:6
            C(i,j)=C(i,j)+.5*(dM(i,j,k)+dM(i,k,j)-dM(j,k,i))*q_dot(k);
        end
    end
end
end
