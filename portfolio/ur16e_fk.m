function [T, points, J] = ur16e_fk(q)
% Nominal UR16e STANDARD DH, metres/radians, base-to-flange (no TCP).
% A_i = Rz(q_i) Tz(d_i) Tx(a_i) Rx(alpha_i). No joint offsets.
% This deliberately does not use the legacy modified-DH functions.
q=q(:); assert(numel(q)==6 && all(isfinite(q)));
a=[0 -.4784 -.36 0 0 0]; d=[.1807 0 0 .17415 .11985 .11655];
alpha=[pi/2 0 0 pi/2 -pi/2 0];
T=eye(4); points=zeros(3,7); axes=zeros(3,6);
for k=1:6
    axes(:,k)=T(1:3,3);
    c=cos(q(k));s=sin(q(k));ca=cos(alpha(k));sa=sin(alpha(k));
    A=[c -s*ca s*sa a(k)*c;s c*ca -c*sa a(k)*s;0 sa ca d(k);0 0 0 1];
    T=T*A; points(:,k+1)=T(1:3,4);
end
J=zeros(6);
for k=1:6
    J(:,k)=[cross(axes(:,k),points(:,7)-points(:,k));axes(:,k)];
end
end
