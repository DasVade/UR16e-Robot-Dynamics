function J = compute_jacobian(q,link_num)
% URDF COM in standard-DH link coordinates; base-frame [linear;angular].
[~,~,com]=urdf_parameters();
a=[0 -.4784 -.36 0 0 0];d=[.1807 0 0 .17415 .11985 .11655];al=[pi/2 0 0 pi/2 -pi/2 0];
T=eye(4);o=zeros(3,6);z=zeros(3,6);
for k=1:link_num
    o(:,k)=T(1:3,4);z(:,k)=T(1:3,3);T=T*dh_transform(q(k),d(k),a(k),al(k));
end
p=T*[com(:,link_num);1];J=zeros(6);
for k=1:link_num,J(:,k)=[cross(z(:,k),p(1:3)-o(:,k));z(:,k)];end
end
