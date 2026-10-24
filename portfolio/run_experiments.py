"""Executed URDF dynamics checks and matched torque-driven tracking experiments."""
import argparse,json,time,platform,importlib.metadata,shutil
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import pybullet as p
from urdf_model import Model,prepare,ROOT,JOINTS,origin
ap=argparse.ArgumentParser();ap.add_argument('--output',default='outputs/executed_01');ap.add_argument('--dt',type=float,default=.00025);args=ap.parse_args()
out=ROOT/args.output;assert not out.exists(),'Choose a fresh output path';out.mkdir(parents=True)
start=time.perf_counter();path=prepare(out/'model');model=Model();p.connect(p.DIRECT);p.setGravity(0,0,-9.81)
def load(path):
    b=p.loadURDF(str(path),useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE)
    for i in range(-1,p.getNumJoints(b)):p.changeDynamics(b,i,linearDamping=0,angularDamping=0,jointDamping=0)
    ids=[i for i in range(p.getNumJoints(b)) if p.getJointInfo(b,i)[2]==p.JOINT_REVOLUTE]
    assert [p.getJointInfo(b,i)[1].decode() for i in ids]==JOINTS
    p.setJointMotorControlArray(b,ids,p.VELOCITY_CONTROL,forces=[0]*6)
    return b,ids
b,ids=load(path);tool=next(i for i in range(p.getNumJoints(b)) if p.getJointInfo(b,i)[12]==b'tool0')
def reset(b,ids,q,v=None):
    for j,a,d in zip(ids,q,np.zeros(6) if v is None else v):p.resetJointState(b,j,float(a),float(d))
def dh(q):
    T=np.eye(4)
    for th,a,d,al in zip(q,[0,-.4784,-.36,0,0,0],[.1807,0,0,.17415,.11985,.11655],[np.pi/2,0,0,np.pi/2,-np.pi/2,0]):
        c,s=np.cos(th),np.sin(th);ca,sa=np.cos(al),np.sin(al);T=T@np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]])
    return T
rng=np.random.default_rng(5170);checks={'samples':20};errors={k:[] for k in ['fk_m','fk_rotation_rad','dh_pose','jacobian','jacobian_fd','mass_symmetry','mass_reference','gravity_reference','bias_reference','ik_position_m','ik_rotation_rad','forward_inverse_acceleration','external_wrench_acceleration']};mineig=[]
for n,q in enumerate(rng.uniform(-1.2,1.2,(20,6))):
    reset(b,ids,q);f=model.frames(q)[0];T=f['tool0'];state=p.getLinkState(b,tool,computeForwardKinematics=True)
    errors['fk_m'].append(float(np.linalg.norm(T[:3,3]-state[4])));errors['fk_rotation_rad'].append(float(Rotation.from_matrix(T[:3,:3]@Rotation.from_quat(state[5]).as_matrix().T).magnitude()))
    errors['dh_pose'].append(float(np.max(abs(dh(q)-np.linalg.inv(f['base'])@T))))
    J=model.jac(q);jbullet=np.vstack(p.calculateJacobian(b,tool,[0,0,0],q.tolist(),[0]*6,[0]*6));errors['jacobian'].append(float(np.max(abs(J-jbullet))))
    for k in range(6):
        e=np.eye(6)[k]*1e-6;Tp=model.frames(q+e)[0]['tool0'];Tm=model.frames(q-e)[0]['tool0'];fd=np.r_[(Tp[:3,3]-Tm[:3,3])/(2e-6),Rotation.from_matrix(Tp[:3,:3]@Tm[:3,:3].T).as_rotvec()/(2e-6)];errors['jacobian_fd'].append(float(np.max(abs(fd-J[:,k]))))
    M=np.array(p.calculateMassMatrix(b,q.tolist()));v=rng.normal(0,.2,6);a=rng.normal(0,.5,6);Mr,c,g=model.dynamics(q,v)
    bias=np.array(p.calculateInverseDynamics(b,q.tolist(),v.tolist(),[0]*6));gb=np.array(p.calculateInverseDynamics(b,q.tolist(),[0]*6,[0]*6))
    errors['mass_symmetry'].append(float(np.max(abs(M-M.T))));mineig.append(float(np.linalg.eigvalsh(M).min()));errors['mass_reference'].append(float(np.max(abs(M-Mr))));errors['gravity_reference'].append(float(np.max(abs(g-gb))));errors['bias_reference'].append(float(np.max(abs(c+g-bias))))
    # Nontrivial near-target numerical full-pose IK; not the legacy analytic solver.
    def residual(z):
        Z=model.frames(z)[0]['tool0'];return np.r_[Z[:3,3]-T[:3,3],Rotation.from_matrix(Z[:3,:3]@T[:3,:3].T).as_rotvec()]
    sol=least_squares(residual,q+rng.normal(0,.15,6),xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=150)
    r=residual(sol.x);errors['ik_position_m'].append(float(np.linalg.norm(r[:3])));errors['ik_rotation_rad'].append(float(np.linalg.norm(r[3:])))
    # Torque-driven forward step, no contacts or damping, inverse-dynamics torque.
    reset(b,ids,q,v);dt=1e-6;p.setTimeStep(dt);tau=p.calculateInverseDynamics(b,q.tolist(),v.tolist(),a.tolist());p.setJointMotorControlArray(b,ids,p.TORQUE_CONTROL,forces=tau);p.stepSimulation();dv=np.array([p.getJointState(b,j)[1] for j in ids]);errors['forward_inverse_acceleration'].append(float(np.max(abs((dv-v)/dt-a))))
    reset(b,ids,q);w=np.array([1.,-.5,.8,.1,-.1,.2]);p.setJointMotorControlArray(b,ids,p.TORQUE_CONTROL,forces=gb.tolist());p.applyExternalForce(b,6,w[:3].tolist(),T[:3,3].tolist(),p.WORLD_FRAME);p.applyExternalTorque(b,6,w[3:].tolist(),p.WORLD_FRAME);p.stepSimulation();actual=np.array([p.getJointState(b,j)[1] for j in ids])/dt;expected=np.linalg.solve(M,J.T@w);errors['external_wrench_acceleration'].append(float(np.max(abs(actual-expected))))
checks.update({k:float(max(v)) for k,v in errors.items()});checks['minimum_mass_eigenvalue']=min(mineig)
assert checks['mass_reference']<1e-10 and checks['gravity_reference']<1e-6 and checks['bias_reference']<1e-5
assert checks['jacobian']<1e-10 and checks['dh_pose']<1e-8 and checks['ik_position_m']<1e-7
assert checks['jacobian_fd']<1e-7 and checks['minimum_mass_eigenvalue']>1e-6 and checks['fk_m']<1e-6 and checks['ik_rotation_rad']<1e-7
assert checks['forward_inverse_acceleration']<1e-5 and checks['external_wrench_acceleration']<1e-5
(out/'verification.json').write_text(json.dumps(checks,indent=2)+'\n')
# Express URDF inertials in the standard-DH frames for corrected MATLAB routines.
q=np.zeros(6);f=model.frames(q)[0];base=f['base'];transforms=[];T=np.eye(4)
for k in range(6):
    # DH frame k transform can be obtained from successive standard factors.
    th=0;a=[0,-.4784,-.36,0,0,0][k];d=[.1807,0,0,.17415,.11985,.11655][k];al=[np.pi/2,0,0,np.pi/2,-np.pi/2,0][k]
    A=np.array([[1,0,0,a],[0,np.cos(al),-np.sin(al),0],[0,np.sin(al),np.cos(al),d],[0,0,0,1]]);T=T@A;transforms.append(base@T)
params=[]
for link,T in zip(['shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link'],transforms):
    e=model.links[link].find('inertial');C=np.linalg.inv(T)@f[link]@origin(e.find('origin'));a=e.find('inertia').attrib;I=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
    params.append({'link':link,'mass':float(e.find('mass').get('value')),'com_DH_m':C[:3,3].tolist(),'inertia_DH':(C[:3,:3]@I@C[:3,:3].T).tolist()})
(out/'reconciled_parameters.json').write_text(json.dumps(params,indent=2)+'\n')
# A 1 kg payload at tool0; controller always retains the nominal robot model.
import xml.etree.ElementTree as ET
xml=ET.parse(path);inert=xml.find(".//link[@name='tool0']/inertial");inert.find('mass').set('value','1');inert.find('inertia').set('ixx','.001');inert.find('inertia').set('iyy','.001');inert.find('inertia').set('izz','.001');payload=out/'model/payload.urdf';xml.write(payload)
dt=args.dt;duration=6.;limits=np.array([330,330,150,56,56,56.]);q0=np.array([0,-1.2,1.2,-.8,.8,0]);q1=np.array([.9,-.5,.5,-.2,1.2,1]);offset=np.array([.08,-.08,.06,-.05,.04,-.03]);kp=np.array([180,180,120,50,30,20]);kd=np.array([30,35,22,8,5,3]);kpa=np.full(6,64.);kda=np.full(6,16.)
config={'dt_s':dt,'duration_s':duration,'joint_order':JOINTS,'q0':q0.tolist(),'q1':q1.tolist(),'initial_offset_rad':offset.tolist(),'torque_limits_Nm':limits.tolist(),'limit_source':'supplied URDF effort fields, treated as simulation bounds, not validated hardware ratings','pd_Kp_Nm_per_rad':kp.tolist(),'pd_Kd_Nm_s_per_rad':kd.tolist(),'ct_Kp_per_s2':kpa.tolist(),'ct_Kd_per_s':kda.tolist(),'payload_kg':1.,'seed':5170,'integration':'PyBullet semi-implicit Euler, torque update every simulation step; contacts disabled','reference':'6 s quintic rest-to-rest joint interpolation','fairness':'same plant/IC/reference/dt/limits; gains have different units and are not claimed equivalent objectives'}
(out/'config.json').write_text(json.dumps(config,indent=2)+'\n');rows=[];p.setTimeStep(dt)
for mismatch in [False,True]:
    for controller in ['pd_gravity','computed_torque']:
        plant,joints=load(payload if mismatch else path);reset(plant,joints,q0+offset)
        for i in range(-1,p.getNumJoints(plant)):p.setCollisionFilterGroupMask(plant,i,0,0)
        t=np.arange(round(duration/dt)+1)*dt;r=t/duration;s=10*r**3-15*r**4+6*r**5;sd=(30*r**2-60*r**3+30*r**4)/duration;sdd=(60*r-180*r**2+120*r**3)/duration**2
        desired=q0+s[:,None]*(q1-q0);vd=sd[:,None]*(q1-q0);ad=sdd[:,None]*(q1-q0);actual=[];vel=[];applied=[];command=[];ee=[]
        for i,now in enumerate(t):
            states=p.getJointStates(plant,joints);q=np.array([x[0] for x in states]);v=np.array([x[1] for x in states]);actual.append(q);vel.append(v);ee.append(model.frames(q)[0]['tool0'][:3,3])
            if i==len(t)-1:break
            if controller=='pd_gravity':raw=kp*(desired[i]-q)+kd*(vd[i]-v)+np.array(p.calculateInverseDynamics(b,q.tolist(),[0]*6,[0]*6))
            else:raw=np.array(p.calculateInverseDynamics(b,q.tolist(),v.tolist(),(ad[i]+kpa*(desired[i]-q)+kda*(vd[i]-v)).tolist()))
            u=np.clip(raw,-limits,limits);command.append(raw);applied.append(u);p.setJointMotorControlArray(plant,joints,p.TORQUE_CONTROL,forces=u.tolist());p.stepSimulation()
        actual=np.array(actual);vel=np.array(vel);applied=np.array(applied);command=np.array(command);error=actual-desired
        assert np.isfinite(actual).all() and abs(actual).max()<10
        name=controller+('_payload' if mismatch else '_nominal');np.savez_compressed(out/(name+'.npz'),time=t,q=actual,v=vel,desired=desired,desired_velocity=vd,desired_acceleration=ad,applied=applied,command=command,ee=np.array(ee))
        np.savetxt(out/(name+'.csv'),np.c_[t,actual,desired,vel,np.vstack([applied,np.full((1,6),np.nan)])],delimiter=',',header='time,q1,q2,q3,q4,q5,q6,qd1,qd2,qd3,qd4,qd5,qd6,v1,v2,v3,v4,v5,v6,tau1,tau2,tau3,tau4,tau5,tau6',comments='')
        rows.append({'case':name,'rmse_rad':np.sqrt(np.mean(error**2,axis=0)).tolist(),'peak_error_rad':abs(error).max(axis=0).tolist(),'peak_applied_Nm':abs(applied).max(axis=0).tolist(),'saturation_fraction':np.mean(abs(command)>limits+1e-10,axis=0).tolist(),'final_error_rad':error[-1].tolist()});p.removeBody(plant)
        print(json.dumps(rows[-1]),flush=True)
(out/'tracking_metrics.json').write_text(json.dumps(rows,indent=2)+'\n')
environment={'platform':platform.platform(),'python':platform.python_version(),'packages':{n:importlib.metadata.version(n) for n in ['numpy','scipy','pybullet','matplotlib','Pillow','trimesh','pycollada']},'elapsed_s':time.perf_counter()-start}
(out/'environment.json').write_text(json.dumps(environment,indent=2)+'\n');print(json.dumps(checks,indent=2));p.disconnect()
