function run_demo(output)
% Base MATLAB only; an unexecuted MATLAB reproduction entry point on this host.
if nargin<1, output=fullfile('outputs',datestr(now,'yyyymmdd_HHMMSS')); end
assert(~exist(output,'dir'),'Choose a fresh output directory.'); mkdir(output);
rng(5170); h=1e-6; worst=0;
for n=1:50
    q=2*rand(6,1)-1; [T,~,J]=ur16e_fk(q);
    for k=1:6
        e=zeros(6,1);e(k)=h;Tp=ur16e_fk(q+e);Tm=ur16e_fk(q-e);
        dR=(Tp(1:3,1:3)-Tm(1:3,1:3))/(2*h); W=dR*T(1:3,1:3)';
        fd=[(Tp(1:3,4)-Tm(1:3,4))/(2*h);W(3,2);W(1,3);W(2,1)];
        worst=max(worst,max(abs(fd-J(:,k))));
    end
end
assert(worst<1e-7,'Jacobian finite-difference check failed.');
t=linspace(0,6,121)'; s=10*(t/6).^3-15*(t/6).^4+6*(t/6).^5;
q0=[0 -1.2 1.2 -.8 .8 0];q1=[.9 -.5 .5 -.2 1.2 1]; q=q0+s*(q1-q0);
xyz=zeros(numel(t),3);sigma=zeros(size(t));
for i=1:numel(t),[T,~,J]=ur16e_fk(q(i,:));xyz(i,:)=T(1:3,4)';sigma(i)=min(svd(J));end
writematrix([t q xyz sigma],fullfile(output,'trajectory.csv'));
f=figure('Color','w');
for i=1:numel(t)
    [~,p]=ur16e_fk(q(i,:));clf;plot3(p(1,:),p(2,:),p(3,:),'o-','LineWidth',3);hold on;
    plot3(xyz(1:i,1),xyz(1:i,2),xyz(1:i,3));grid on;axis equal;
    xlim([-1 1]);ylim([-1 1]);zlim([-.3 1.2]);view(40,25);
    xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');title(sprintf('Kinematic joint motion | t=%.2f s',t(i)));
    drawnow;[im,map]=rgb2ind(frame2im(getframe(f)),256);
    if i==1,imwrite(im,map,fullfile(output,'motion.gif'),'gif','LoopCount',inf,'DelayTime',.05);
    else,imwrite(im,map,fullfile(output,'motion.gif'),'gif','WriteMode','append','DelayTime',.05);end
end
fprintf('Maximum Jacobian residual: %.3g\n',worst);
fid=fopen(fullfile(output,'verification.json'),'w');fprintf(fid,'%s',jsonencode(struct('execution','MATLAB','max_jacobian_residual',worst)));fclose(fid);
end
