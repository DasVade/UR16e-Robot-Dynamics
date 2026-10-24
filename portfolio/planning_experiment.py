"""Secondary Python adaptation of the existing joint-space RRT/shortcut logic.
Not execution of the original MATLAB planner; uses Bullet collision meshes.
"""
import json,time,argparse
import numpy as np
import pybullet as p
from urdf_model import ROOT,prepare
ap=argparse.ArgumentParser();ap.add_argument('--output',default='outputs/planning_01');args=ap.parse_args()
out=ROOT/args.output;assert not out.exists();out.mkdir(parents=True)
path=prepare(out/'model');p.connect(p.DIRECT);b=p.loadURDF(str(path),useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE);ids=[i for i in range(p.getNumJoints(b)) if p.getJointInfo(b,i)[2]==p.JOINT_REVOLUTE]
# Declared new test scene in base_link metres; no ground-plane constraint.
obstacles=[p.createMultiBody(0,p.createCollisionShape(p.GEOM_BOX,halfExtents=[.07,.07,.10]),basePosition=[.35,.05,.40]),p.createMultiBody(0,p.createCollisionShape(p.GEOM_SPHERE,radius=.08),basePosition=[-.25,.50,.30])]
def collision(q):
    for j,v in zip(ids,q):p.resetJointState(b,j,float(v))
    if any(p.getClosestPoints(b,o,0) for o in obstacles):return True
    # Explicit nonadjacent moving-link pairs; adjacent/fixed helper contacts ignored.
    return any(p.getClosestPoints(b,b,0,linkIndexA=i,linkIndexB=j) for i in range(0,7) for j in range(i+3,7))
def edge(a,b,spacing=.04):
    n=max(1,int(np.ceil(np.linalg.norm(b-a)/spacing)))
    return all(not collision(a+(b-a)*s) for s in np.linspace(0,1,n+1))
start=np.array([0,-1.2,1.2,-.8,.8,0]);goal=np.array([.9,-.5,.5,-.2,1.2,1]);lo=np.minimum(start,goal)-.5;hi=np.maximum(start,goal)+.5
rows=[]
for seed in [5170,5171,5172]:
    rng=np.random.default_rng(seed);begin=time.perf_counter();nodes=[start];parents=[-1];path=None;reason=None
    if collision(start) or collision(goal):reason='start or goal collision'
    else:
        for iteration in range(1500):
            target=goal if rng.random()<.2 else rng.uniform(lo,hi);dist=np.linalg.norm(np.array(nodes)-target,axis=1);near=int(np.argmin(dist))
            if dist[near]<1e-12:continue
            q=nodes[near]+min(.18,dist[near])*(target-nodes[near])/dist[near]
            if not edge(nodes[near],q):continue
            nodes.append(q);parents.append(near)
            if np.linalg.norm(q-goal)<.25 and edge(q,goal):
                path=[goal];idx=len(nodes)-1
                while idx>=0:path.append(nodes[idx]);idx=parents[idx]
                path=np.array(path[::-1]);break
    elapsed=time.perf_counter()-begin
    if path is not None:
        raw=path.copy()
        for _ in range(50):
            a,c=sorted(rng.choice(len(path),2,replace=False))
            if c>a+1 and edge(path[a],path[c]):path=np.r_[path[:a+1],path[c:]]
        # Independently recheck at 4x finer joint-space spacing after smoothing.
        valid=all(edge(a,c,.01) for a,c in zip(path[:-1],path[1:]));assert valid
        np.savez_compressed(out/f'seed{seed}.npz',raw=raw,smoothed=path)
        rows.append({'seed':seed,'success':True,'planning_seconds':elapsed,'raw_length_rad':float(np.linalg.norm(np.diff(raw,axis=0),axis=1).sum()),'smoothed_length_rad':float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),'fine_edge_validation':valid})
    else:rows.append({'seed':seed,'success':False,'planning_seconds':elapsed,'reason':reason or 'iteration budget exhausted'})
config={'source':'Python adaptation of original RRT extension and shortcut ideas, not MATLAB execution','seeds':[5170,5171,5172],'edge_spacing_rad':.04,'validation_spacing_rad':.01,'metric':'Euclidean joint-space radians','joint_box_lower':lo.tolist(),'joint_box_upper':hi.tolist(),'obstacles':[{'box_center_m':[.35,.05,.40],'half_extents_m':[.07,.07,.10]},{'sphere_center_m':[-.25,.50,.30],'radius_m':.08}],'self_pairs':'indices 0..6 with index separation >=3; other pairs excluded','limitations':'finite edge sampling and supplied convex collision meshes do not certify continuous clearance or hardware safety','results':rows}
(out/'planning.json').write_text(json.dumps(config,indent=2)+'\n');(ROOT/'docs/executed/planning.json').write_text(json.dumps(config,indent=2)+'\n');print(json.dumps(rows,indent=2));p.disconnect()
