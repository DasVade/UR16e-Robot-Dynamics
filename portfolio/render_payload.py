"""Nominal/payload mesh comparison, original recorded CT states and timing."""
import argparse,subprocess
from pathlib import Path
import numpy as np
import pybullet as p
from PIL import Image,ImageDraw
from robot import Robot
from urdf_model import ROOT
ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);args=ap.parse_args();run=ROOT/args.run;dest=ROOT/'docs/migration';dest.mkdir(exist_ok=True)
robot=Robot(run/'model');traces=[np.load(run/(n+'.npz')) for n in ['computed_torque_nominal','computed_torque_payload']];assert np.array_equal(traces[0]['time'],traces[1]['time'])
view=p.computeViewMatrixFromYawPitchRoll([.25,.35,.4],1.8,-45,-28,0,2);proj=p.computeProjectionMatrixFOV(45,480/360,.05,5);frames=[]
stride=round(.1/(traces[0]['time'][1]-traces[0]['time'][0]))
for i in range(0,len(traces[0]['time']),stride):
    canvas=Image.new('RGB',(960,400),'white');draw=ImageDraw.Draw(canvas)
    for panel,(d,label) in enumerate(zip(traces,['Nominal CT','CT with unmodeled 1 kg payload'])):
        robot.reset(d['q'][i]);rgb=np.array(p.getCameraImage(480,360,view,proj,renderer=p.ER_TINY_RENDERER)[2],dtype=np.uint8).reshape(360,480,4)[:,:,:3];canvas.paste(Image.fromarray(rgb),(panel*480,30));draw.text((panel*480+10,10),f'{label} | {d["time"][i]:.1f} s',fill='black')
    draw.text((12,385),'Torque-driven recorded states | identical view, scale and clock | geometry unchanged; payload is an inertial model addition',fill='black');frames.append(canvas)
frames[0].save(dest/'payload-poster.png');frames[len(frames)//2].save(dest/'payload-preview.png');frames[0].save(dest/'payload.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x400','-framerate','10','-i','-','-an','-c:v','libx264','-crf','24','-pix_fmt','yuv420p','-movflags','+faststart',str(dest/'payload.mp4')],stdin=subprocess.PIPE)
for im in frames:proc.stdin.write(im.tobytes())
proc.stdin.close();assert proc.wait()==0;p.disconnect()
