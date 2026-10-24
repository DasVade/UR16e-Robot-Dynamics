function G = compute_gravity(q,masses,g)
% Compensation G=dU/dq, U=sum(m*g*z), g is positive gravity magnitude.
G=zeros(6,1);
for k=1:6
    J=compute_jacobian(q,k);G=G+masses(k)*J(1:3,:)'*[0;0;g];
end
end
