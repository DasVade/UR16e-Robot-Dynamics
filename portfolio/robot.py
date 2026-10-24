"""Primary six-joint URDF robot interface; SI units, world/base_link frame."""
from pathlib import Path
import numpy as np
import pybullet as p
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from urdf_model import Model,prepare,ROOT,JOINTS

class Robot:
    def __init__(self,workspace,urdf=None):
        if not p.isConnected():p.connect(p.DIRECT)
        self.model=Model();self.path=Path(urdf) if urdf else prepare(workspace)
        self.body=p.loadURDF(str(self.path),useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.ids=[i for i in range(p.getNumJoints(self.body)) if p.getJointInfo(self.body,i)[2]==p.JOINT_REVOLUTE]
        assert [p.getJointInfo(self.body,i)[1].decode() for i in self.ids]==JOINTS
        self.tool=next(i for i in range(p.getNumJoints(self.body)) if p.getJointInfo(self.body,i)[12]==b'tool0')
        js={j.get('name'):j for j in self.model.joints}
        self.lower=np.array([float(js[n].find('limit').get('lower')) for n in JOINTS]);self.upper=np.array([float(js[n].find('limit').get('upper')) for n in JOINTS])
        self.limits=np.array([float(js[n].find('limit').get('effort')) for n in JOINTS]);self.velocity_limits=np.array([float(js[n].find('limit').get('velocity')) for n in JOINTS])
        p.setJointMotorControlArray(self.body,self.ids,p.VELOCITY_CONTROL,forces=[0]*6)
        for i in range(-1,p.getNumJoints(self.body)):
            p.changeDynamics(self.body,i,linearDamping=0,angularDamping=0,jointDamping=0)
            p.setCollisionFilterGroupMask(self.body,i,0,0)
        p.setGravity(0,0,-9.81)
    def reset(self,q,v=None):
        for j,a,d in zip(self.ids,q,np.zeros(6) if v is None else v):p.resetJointState(self.body,j,float(a),float(d))
    def state(self):
        s=p.getJointStates(self.body,self.ids);return np.array([x[0] for x in s]),np.array([x[1] for x in s])
    def fk(self,q):return self.model.frames(np.asarray(q))[0]['tool0']
    def jacobian(self,q):return self.model.jac(np.asarray(q))
    def ik(self,target,initial):
        def error(q):
            T=self.fk(q);return np.r_[T[:3,3]-target[:3,3],Rotation.from_matrix(T[:3,:3]@target[:3,:3].T).as_rotvec()]
        sol=least_squares(error,np.clip(initial,self.lower+1e-8,self.upper-1e-8),bounds=(self.lower,self.upper),xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=300)
        return sol.x,{'solver_success':bool(sol.success),'position_residual_m':float(np.linalg.norm(error(sol.x)[:3])),'orientation_residual_rad':float(np.linalg.norm(error(sol.x)[3:]))}
    def mass(self,q):return np.array(p.calculateMassMatrix(self.body,np.asarray(q).tolist()))
    def inverse(self,q,v,a,wrench=None):
        tau=np.array(p.calculateInverseDynamics(self.body,np.asarray(q).tolist(),np.asarray(v).tolist(),np.asarray(a).tolist()))
        if wrench is not None:tau-=self.jacobian(q).T@wrench
        return tau
    def gravity(self,q):return self.inverse(q,np.zeros(6),np.zeros(6))
    def velocity_terms(self,q,v):return self.inverse(q,v,np.zeros(6))-self.gravity(q)
    def forward(self,q,v,tau,wrench=None):
        rhs=np.asarray(tau)-self.inverse(q,v,np.zeros(6))
        if wrench is not None:rhs+=self.jacobian(q).T@wrench
        return np.linalg.solve(self.mass(q),rhs)
    def step(self,tau):p.setJointMotorControlArray(self.body,self.ids,p.TORQUE_CONTROL,forces=np.asarray(tau).tolist());p.stepSimulation()

def reference(path,dt,duration_per_rad=3.):
    """C2 rest-to-rest quintic on EACH validated straight joint-space edge.
    Stops at knots; does not fit a spline that could leave collision-free edges.
    """
    path=np.asarray(path);ts=[];qs=[];vs=[];acc=[];now=0
    for a,b in zip(path[:-1],path[1:]):
        length=np.linalg.norm(b-a);duration=np.ceil(max(1.5,duration_per_rad*length)/.01)*.01
        n=int(round(duration/dt));assert np.isclose(n*dt,duration)
        r=np.arange(n)*dt/duration
        s=10*r**3-15*r**4+6*r**5;sd=(30*r**2-60*r**3+30*r**4)/duration;sdd=(60*r-180*r**2+120*r**3)/duration**2
        ts.extend(now+np.arange(n)*dt);qs.extend(a+s[:,None]*(b-a));vs.extend(sd[:,None]*(b-a));acc.extend(sdd[:,None]*(b-a));now+=duration
    return np.r_[ts,now],np.vstack([qs,path[-1]]),np.vstack([vs,np.zeros(6)]),np.vstack([acc,np.zeros(6)])
