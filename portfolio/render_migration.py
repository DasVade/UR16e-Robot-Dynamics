"""Render integrated planned/dynamic traces with the exact checked obstacles."""
import argparse,json,subprocess,shutil
from pathlib import Path
import numpy as np
import pybullet as p
from PIL import Image,ImageDraw
from robot import Robot
from planner import CollisionScene
from urdf_model import ROOT
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);args=ap.parse_args();run=ROOT/args.run;d=np.load(run/'execution.npz');dest=ROOT/'docs/migration';dest.mkdir(exist_ok=True)
for name in ['config.json','metrics.json']:shutil.copyfile(run/name,dest/name)
path=np.load(run/'path_seed5170.npz');(dest/'path.json').write_text(json.dumps({k:v.tolist() for k,v in dict(path).items()},indent=2)+'\n')
stride=round(.02/(d['time'][1]-d['time'][0]));ix=np.r_[np.arange(0,len(d['time'])-1,stride),len(d['time'])-1];u=np.vstack([d['applied'],np.full((1,6),np.nan)])
np.savetxt(dest/'execution_20ms.csv',np.c_[d['time'][ix],d['q'][ix],d['desired'][ix],u[ix]],delimiter=',',header='time,q1,q2,q3,q4,q5,q6,desired1,desired2,desired3,desired4,desired5,desired6,tau1,tau2,tau3,tau4,tau5,tau6',comments='')
fig,axes=plt.subplots(2,3,figsize=(10,5),sharex=True)
for j,ax in enumerate(axes.flat):ax.plot(d['time'],d['q'][:,j]-d['desired'][:,j]);ax.set(title=f'Joint {j+1}',xlabel='Time (s)',ylabel='Error (rad)')
fig.suptitle('Dynamic tracking of the RRT-generated timed path');fig.tight_layout();fig.savefig(dest/'integrated_errors.png',dpi=140);plt.close(fig)
scene=CollisionScene(run/'model');robot=scene.robot
view=p.computeViewMatrixFromYawPitchRoll([.25,.35,.4],2.05,-45,-28,0,2);proj=p.computeProjectionMatrixFOV(48,720/480,.05,5);V=np.array(view).reshape(4,4,order='F');P=np.array(proj).reshape(4,4,order='F')
def project(points):
    x=(P@V@np.c_[points,np.ones(len(points))].T).T;x=x[:,:3]/x[:,3:4];return [((v[0]+1)*360,(1-v[1])*240) for v in x]
times=np.arange(0,float(d['time'][-1]),.1);times=np.r_[times,d['time'][-1]];indices=np.searchsorted(d['time'],times);desired=np.array([robot.fk(q)[:3,3] for q in d['desired'][indices]])
for mode in ['planning','execution']:
    frames=[]
    for k,i in enumerate(indices):
        q=d['desired'][i] if mode=='planning' else d['q'][i];robot.reset(q)
        rgb=np.array(p.getCameraImage(720,480,view,proj,renderer=p.ER_TINY_RENDERER)[2],dtype=np.uint8).reshape(480,720,4)[:,:,:3];im=Image.fromarray(rgb);draw=ImageDraw.Draw(im)
        planned=project(desired)
        for n in range(0,len(planned)-1,2):draw.line(planned[n:n+2],fill=(225,100,0),width=2)
        actual=project(np.array([robot.fk(q)[:3,3] for q in (d['desired'] if mode=='planning' else d['q'])[indices[:k+1]]]))
        if len(actual)>1:draw.line(actual,fill=(0,80,220),width=3)
        for pt,text in [(planned[0],'START'),(planned[-1],'GOAL')]:draw.ellipse((pt[0]-4,pt[1]-4,pt[0]+4,pt[1]+4),fill=(0,120,0));draw.text((pt[0]+5,pt[1]),text,fill='black')
        draw.rectangle((0,0,720,35),fill='white');draw.text((12,10),('Planned-path playback (kinematic)' if mode=='planning' else 'Torque-driven tracking of planned path')+f' | t={d["time"][i]:.2f} s',fill='black')
        draw.text((12,460),'Exact checked obstacles | orange: desired EE path | blue: played/actual EE path',fill='black');frames.append(im)
    frames[0].save(dest/(mode+'-poster.png'));frames[len(frames)//2].save(dest/(mode+'-preview.png'));frames[0].save(dest/(mode+'.gif'),save_all=True,append_images=frames[1:],duration=100,loop=0)
    # Encode the original 10 fps frames as H.264; no motion-speed changes.
    process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','720x480','-framerate','10','-i','-','-an','-c:v','libx264','-crf','24','-pix_fmt','yuv420p','-movflags','+faststart',str(dest/(mode+'.mp4'))],stdin=subprocess.PIPE)
    for im in frames:process.stdin.write(im.tobytes())
    process.stdin.close();assert process.wait()==0
p.disconnect()
