"""Curate verified run outputs and render actual URDF meshes at simulated states."""
import argparse,json,shutil
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from urdf_model import ROOT,Model,prepare
import pybullet as p
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--half-step',required=True);args=ap.parse_args();run=ROOT/args.run;half=ROOT/args.half_step;dest=ROOT/'docs/executed';dest.mkdir(exist_ok=True)
for name in ['config.json','environment.json','verification.json','tracking_metrics.json','reconciled_parameters.json']:shutil.copyfile(run/name,dest/name)
shutil.copyfile(half/'verification.json',dest/'half_step_verification.json')
metrics=json.loads((run/'tracking_metrics.json').read_text());convergence=[]
for row in metrics:
    name=row['case'];d=np.load(run/(name+'.npz'));h=np.load(half/(name+'.npz'))
    delta=float(np.max(abs(d['q']-h['q'][::2])));convergence.append({'case':name,'max_joint_state_difference_rad':delta});assert delta<.0003
    stride=round(.02/(d['time'][1]-d['time'][0]));ix=np.arange(0,len(d['time']),stride);u=np.vstack([d['applied'],np.full((1,6),np.nan)])
    np.savetxt(dest/(name+'_20ms.csv'),np.c_[d['time'][ix],d['q'][ix],d['desired'][ix],u[ix]],delimiter=',',header='time_s,q1,q2,q3,q4,q5,q6,desired1,desired2,desired3,desired4,desired5,desired6,tau1_Nm,tau2_Nm,tau3_Nm,tau4_Nm,tau5_Nm,tau6_Nm',comments='')
(dest/'convergence.json').write_text(json.dumps(convergence,indent=2)+'\n')
for suffix,title in [('nominal','Nominal model'),('payload','Unmodeled 1 kg payload')]:
    fig,axes=plt.subplots(2,3,figsize=(10,5),sharex=True)
    for controller,label in [('pd_gravity','PD + gravity'),('computed_torque','Computed torque')]:
        d=np.load(run/(controller+'_'+suffix+'.npz'))
        for j,ax in enumerate(axes.flat):ax.plot(d['time'],d['q'][:,j]-d['desired'][:,j],label=label,lw=1);ax.set(title=f'Joint {j+1}',xlabel='Time (s)',ylabel='Error (rad)')
    axes.flat[0].legend(fontsize=8);fig.suptitle(title+' | torque-driven nonlinear simulation');fig.tight_layout();fig.savefig(dest/('tracking_'+suffix+'.png'),dpi=140);plt.close(fig)
fig,axes=plt.subplots(2,3,figsize=(10,5),sharex=True)
for controller,label in [('pd_gravity','PD + gravity'),('computed_torque','Computed torque')]:
    d=np.load(run/(controller+'_nominal.npz'))
    for j,ax in enumerate(axes.flat):ax.plot(d['time'][:-1],d['applied'][:,j],label=label,lw=1);ax.set(title=f'Joint {j+1}',xlabel='Time (s)',ylabel='Applied torque (Nm)')
axes.flat[0].legend(fontsize=8);fig.suptitle('Applied torque | nominal model');fig.tight_layout();fig.savefig(dest/'torques.png',dpi=140);plt.close(fig)
p.connect(p.DIRECT);path=prepare(run/'model');b=p.loadURDF(str(path),useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE);joints=[i for i in range(p.getNumJoints(b)) if p.getJointInfo(b,i)[2]==p.JOINT_REVOLUTE]
model=Model();view=p.computeViewMatrixFromYawPitchRoll([0,0,.45],1.9,40,-25,0,2);proj=p.computeProjectionMatrixFOV(45,480/360,.05,5);V=np.array(view).reshape(4,4,order='F');P=np.array(proj).reshape(4,4,order='F')
def project(points):
    h=np.c_[points,np.ones(len(points))];c=(P@V@h.T).T;c=c[:,:3]/c[:,3:4];return [(float((v[0]+1)*240),float((1-v[1])*180)) for v in c]
frames=[];traces=[np.load(run/(c+'_nominal.npz')) for c in ['pd_gravity','computed_torque']];indices=np.arange(0,len(traces[0]['time']),round(.1/(traces[0]['time'][1]-traces[0]['time'][0])))
desired=np.array([model.frames(q)[0]['tool0'][:3,3] for q in traces[0]['desired'][indices]])
for k,i in enumerate(indices):
    canvas=Image.new('RGB',(960,400),'white');draw=ImageDraw.Draw(canvas)
    for panel,(d,label) in enumerate(zip(traces,['PD + gravity','Computed torque'])):
        for j,q in zip(joints,d['q'][i]):p.resetJointState(b,j,float(q))
        rgb=np.array(p.getCameraImage(480,360,view,proj,renderer=p.ER_TINY_RENDERER)[2],dtype=np.uint8).reshape(360,480,4)[:,:,:3]
        im=Image.fromarray(rgb);line=ImageDraw.Draw(im);dpath=project(desired)
        for n in range(0,len(dpath)-1,2):line.line(dpath[n:n+2],fill=(220,100,0),width=2)
        actual=project(d['ee'][indices[:k+1]])
        if len(actual)>1:line.line(actual,fill=(0,80,230),width=2)
        canvas.paste(im,(panel*480,30));draw.text((panel*480+15,8),f'{label} | t={d["time"][i]:.1f} s',fill='black')
    draw.text((15,385),'Torque-driven simulation | supplied UR16e meshes | orange: desired path, blue: actual path',fill='black');frames.append(canvas)
frames[0].save(dest/'tracking.gif',save_all=True,append_images=frames[1:],duration=100,loop=0);frames[len(frames)//2].save(dest/'tracking_preview.png');p.disconnect()
print(json.dumps(convergence,indent=2))
