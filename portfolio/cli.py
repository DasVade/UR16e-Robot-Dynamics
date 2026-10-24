"""One command-line entry point for the runnable Python project."""
import argparse,json,sys,subprocess,os,platform,time
from pathlib import Path
import numpy as np
import pybullet as p
from robot import Robot,reference
from planner import CollisionScene,rrt,DEFAULT_SCENE
from urdf_model import ROOT,JOINTS
from controllers import command as controller_command

def integrate(out,dt):
    out.mkdir(parents=True);scene=CollisionScene(out/'model');start=np.array([0,-1.2,1.2,-.8,.8,0]);goal=np.array([.9,-.5,.5,-.2,1.2,1])
    rows=[];paths=[]
    for seed in [5170,5171,5172]:
        raw,path,row=rrt(scene,start,goal,seed);rows.append(row);paths.append(path);np.savez_compressed(out/f'path_seed{seed}.npz',raw=raw,smoothed=path);print(json.dumps(row),flush=True)
    path=paths[0];t,qd,vd,ad=reference(path,dt);plant=Robot(out/'model');control=Robot(out/'model');offset=np.array([.01,-.01,.01,-.005,.005,-.005]);plant.reset(start+offset);p.setTimeStep(dt)
    qlog=[];vlog=[];command=[];applied=[];ee=[]
    for i,now in enumerate(t):
        q,v=plant.state();qlog.append(q);vlog.append(v)
        if i==len(t)-1:break
        tau=controller_command('computed_torque',q,v,qd[i],vd[i],ad[i],control.inverse,control.gravity);u=np.clip(tau,-plant.limits,plant.limits);command.append(tau);applied.append(u);plant.step(u)
    qlog=np.array(qlog);vlog=np.array(vlog);command=np.array(command);applied=np.array(applied);assert np.isfinite(qlog).all()
    # Every 5 ms plus final sample; each reference/actual interval also tested at
    # <= .005 rad Euclidean spacing. These are sampled checks, not certificates.
    stride=round(.005/dt);ix=np.unique(np.r_[np.arange(0,len(t),stride),len(t)-1]);checks={}
    for label,trace in [('reference',qd),('actual',qlog)]:
        distances=np.array([scene.clearance(q) for q in trace[ix]])
        edges=[scene.edge(a,b,.005,margin=0) for a,b in zip(trace[ix[:-1]],trace[ix[1:]])]
        checks[label]={'minimum_sampled_clearance_m':float(distances.min()),'colliding_samples':int(np.sum(distances<0)),'invalid_dense_edges':int(np.sum(~np.array(edges))),'position_limit_violations':int(np.sum(np.any((trace<plant.lower)|(trace>plant.upper),axis=1)))}
    metrics={'dt_s':dt,'duration_s':float(t[-1]),'rmse_per_joint_rad':np.sqrt(np.mean((qlog-qd)**2,axis=0)).tolist(),'pooled_rmse_rad':float(np.sqrt(np.mean((qlog-qd)**2))),'peak_error_per_joint_rad':abs(qlog-qd).max(axis=0).tolist(),'peak_applied_Nm':abs(applied).max(axis=0).tolist(),'saturation_fraction':np.mean(abs(command)>plant.limits+1e-10,axis=0).tolist(),'actuator_limit_violations':int(np.sum(np.any(abs(applied)>plant.limits+1e-10,axis=1))),'reference_velocity_limit_violations':int(np.sum(np.any(abs(vd)>plant.velocity_limits,axis=1))),'actual_velocity_limit_violations':int(np.sum(np.any(abs(vlog)>plant.velocity_limits,axis=1))),'validation':checks}
    config={'scene':DEFAULT_SCENE,'planning':rows,'joint_order':JOINTS,'initial_offset_rad':offset.tolist(),'controller':'CT = ID_nom(q,v,ad+64*(qd-q)+16*(vd-v)); applied torque clipped','torque_bounds_Nm':plant.limits.tolist(),'joint_lower_rad':plant.lower.tolist(),'joint_upper_rad':plant.upper.tolist(),'velocity_bounds_rad_s':plant.velocity_limits.tolist(),'collision_check_period_s':.005,'additional_edge_spacing_rad':.005,'self_collision_policy':'indices 0..6 separated by >=3; adjacent/fixed helper pairs excluded','integration':'PyBullet semi-implicit Euler; contacts disabled; obstacles checked independently','time_parameterization':'piecewise quintic along smoothed RRT joint-space edges, zero velocity and acceleration at every knot','reference_samples':len(t),'platform':platform.platform(),'python':platform.python_version()}
    np.savez_compressed(out/'execution.npz',time=t,q=qlog,v=vlog,desired=qd,desired_velocity=vd,desired_acceleration=ad,command=command,applied=applied,path=path)
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n');(out/'config.json').write_text(json.dumps(config,indent=2)+'\n');print(json.dumps(metrics,indent=2));p.disconnect()
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('task',choices=['kinematics','compare','payload','plan-track','verify','visualize']);ap.add_argument('--output');ap.add_argument('--run');ap.add_argument('--dt',type=float,default=.00025);args=ap.parse_args()
    if args.task in ['kinematics','compare','payload','plan-track']:
        if not args.output:ap.error('--output is required; choose a fresh directory')
        out=ROOT/args.output
        if out.exists():raise FileExistsError(out)
    if args.task in ['compare','payload']:
        cmd=[sys.executable,str(ROOT/'portfolio/run_experiments.py'),'--output',args.output,'--dt',str(args.dt)]
        if args.task=='payload':cmd+=['--payload-only']
        subprocess.run(cmd,check=True)
    elif args.task=='plan-track':integrate(out,args.dt)
    elif args.task=='kinematics':
        out.mkdir(parents=True);robot=Robot(out/'model');q=np.array([.2,-1,.7,-.4,.9,.1]);target=robot.fk(q);solution,metrics=robot.ik(target,q+.1);M=robot.mass(q);v=np.full(6,.1);a=np.full(6,.2);tau=robot.inverse(q,v,a)
        metrics.update({'q_rad':q.tolist(),'fk':target.tolist(),'jacobian':robot.jacobian(q).tolist(),'ik_q_rad':solution.tolist(),'mass_matrix':M.tolist(),'gravity_Nm':robot.gravity(q).tolist(),'velocity_terms_Nm':robot.velocity_terms(q,v).tolist(),'inverse_Nm':tau.tolist(),'forward_residual_rad_s2':float(np.max(abs(robot.forward(q,v,tau)-a)))})
        (out/'kinematics.json').write_text(json.dumps(metrics,indent=2)+'\n');print(json.dumps(metrics,indent=2));p.disconnect()
    elif args.task=='verify':
        env=dict(os.environ)
        if args.run:env['UR16E_RUN']=args.run
        subprocess.run([sys.executable,str(ROOT/'portfolio/test_executed.py')],env=env,check=True);subprocess.run([sys.executable,str(ROOT/'portfolio/test_migration.py')],check=True)
    elif args.task=='visualize':
        if not args.run:ap.error('--run is required')
        subprocess.run([sys.executable,str(ROOT/'portfolio/render_migration.py'),'--run',args.run],check=True)
if __name__=='__main__':main()
