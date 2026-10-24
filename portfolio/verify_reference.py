"""Independent nominal kinematics check/render, NOT MATLAB/Simulink execution.
Python 3.12; numpy, scipy, matplotlib, Pillow. The primary demo is run_demo.m.
"""
from pathlib import Path
import json,os
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'outputs/mpl'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from scipy.spatial.transform import Rotation
OUT=ROOT/'docs/results';OUT.mkdir(parents=True,exist_ok=True)
A=[0,-.4784,-.36,0,0,0];D=[.1807,0,0,.17415,.11985,.11655];AL=[np.pi/2,0,0,np.pi/2,-np.pi/2,0]
def fk(q):
    T=np.eye(4);points=[T[:3,3].copy()];axes=[]
    for theta,a,d,alpha in zip(q,A,D,AL):
        axes.append(T[:3,2].copy());c,s=np.cos(theta),np.sin(theta);ca,sa=np.cos(alpha),np.sin(alpha)
        T=T@np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]])
        points.append(T[:3,3].copy())
    p=np.array(points);z=np.array(axes)
    J=np.vstack([np.cross(z,p[-1]-p[:-1]).T,z.T])
    return T,p,J
rng=np.random.default_rng(5170);h=1e-6;errors=[]
for q in rng.uniform(-1,1,(50,6)):
    T,p,J=fk(q)
    assert np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
    for k in range(6):
        e=np.eye(6)[k]*h;Tp=fk(q+e)[0];Tm=fk(q-e)[0]
        fd=np.r_[(Tp[:3,3]-Tm[:3,3])/(2*h),Rotation.from_matrix(Tp[:3,:3]@Tm[:3,:3].T).as_rotvec()/(2*h)]
        errors.append(float(abs(fd-J[:,k]).max()))
assert max(errors)<1e-7
t=np.linspace(0,6,121);s=10*(t/6)**3-15*(t/6)**4+6*(t/6)**5
q0=np.array([0,-1.2,1.2,-.8,.8,0]);q1=np.array([.9,-.5,.5,-.2,1.2,1]);qs=q0+s[:,None]*(q1-q0)
traces=[fk(q) for q in qs];xyz=np.array([d[0][:3,3] for d in traces]);sig=np.array([np.linalg.svd(d[2],compute_uv=False)[-1] for d in traces])
np.savetxt(OUT/'reference_trajectory.csv',np.c_[t,qs,xyz,sig],delimiter=',',header='time_s,q1,q2,q3,q4,q5,q6,x_m,y_m,z_m,sigma_min_mixed_units',comments='')
metrics={'execution':'Independent Python reference; MATLAB and Simulink NOT executed','seed':5170,'configurations_checked':50,'max_jacobian_residual':max(errors),'minimum_mixed_unit_jacobian_singular_value':float(sig.min()),'sampled_path_length_m':float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),'duration_s':6,'samples':121}
(OUT/'verification.json').write_text(json.dumps(metrics,indent=2)+'\n')
fig=plt.figure(figsize=(6,4.5));ax=fig.add_subplot(projection='3d');body,=ax.plot([],[],[],'o-',lw=3);trail,=ax.plot([],[],[],lw=1)
ax.set(xlim=(-1,1),ylim=(-1,1),zlim=(-.3,1.2),xlabel='x (m)',ylabel='y (m)',zlabel='z (m)');ax.view_init(25,40)
title=ax.set_title('Nominal kinematics (Python reference) | 0.00 s',fontsize=10);fig.tight_layout()
def frame(i):
    p=traces[i][1];body.set_data_3d(p[:,0],p[:,1],p[:,2]);trail.set_data_3d(xyz[:i+1,0],xyz[:i+1,1],xyz[:i+1,2]);title.set_text(f'Nominal kinematics (Python reference) | {t[i]:.2f} s')
FuncAnimation(fig,frame,frames=range(0,121,2),interval=100).save(OUT/'motion.gif',writer=PillowWriter(fps=10));plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(9,3.5));axes[0].plot(t,qs);axes[0].set(xlabel='Time (s)',ylabel='Joint angle (rad)');axes[1].plot(t,xyz);axes[1].set(xlabel='Time (s)',ylabel='Flange position (m)');axes[1].legend(['x','y','z']);fig.tight_layout();fig.savefig(OUT/'trajectory.png',dpi=150)
print(json.dumps(metrics,indent=2))
