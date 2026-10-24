"""Reusable RRT and finite-resolution mesh collision checking."""
import time
import numpy as np
import pybullet as p
from robot import Robot
DEFAULT_SCENE=[{'type':'box','center':[.635,.570,.482],'half_extents':[.055,.055,.07]}, {'type':'sphere','center':[-.25,.50,.30],'radius':.08}]
class CollisionScene:
    def __init__(self,workspace,scene=DEFAULT_SCENE):
        self.robot=Robot(workspace);self.spec=scene;self.obstacles=[]
        for o in scene:
            if o['type']=='box':shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=o['half_extents']);vis=p.createVisualShape(p.GEOM_BOX,halfExtents=o['half_extents'],rgbaColor=[.85,.25,.12,1])
            else:shape=p.createCollisionShape(p.GEOM_SPHERE,radius=o['radius']);vis=p.createVisualShape(p.GEOM_SPHERE,radius=o['radius'],rgbaColor=[.85,.65,.10,1])
            self.obstacles.append(p.createMultiBody(0,shape,vis,basePosition=o['center']))
        self.pairs=[(i,j) for i in range(7) for j in range(i+3,7)]
    def clearance(self,q):
        self.robot.reset(q);distances=[1.]
        for o in self.obstacles:distances.extend(x[8] for x in p.getClosestPoints(self.robot.body,o,1.))
        for i,j in self.pairs:distances.extend(x[8] for x in p.getClosestPoints(self.robot.body,self.robot.body,1.,linkIndexA=i,linkIndexB=j))
        return min(distances)
    def valid(self,q,margin=.015):
        return bool(np.all(q>=self.robot.lower) and np.all(q<=self.robot.upper) and self.clearance(q)>=margin)
    def edge(self,a,b,spacing=.04,margin=.015):
        n=max(1,int(np.ceil(np.linalg.norm(b-a)/spacing)))
        return all(self.valid(a+s*(b-a),margin) for s in np.linspace(0,1,n+1))
def rrt(scene,start,goal,seed=5170):
    start=np.asarray(start);goal=np.asarray(goal);rng=np.random.default_rng(seed);begin=time.perf_counter()
    if not scene.valid(start) or not scene.valid(goal):raise ValueError('Invalid endpoint under declared margin/pair policy')
    lo=np.maximum(np.minimum(start,goal)-.8,scene.robot.lower);hi=np.minimum(np.maximum(start,goal)+.8,scene.robot.upper);nodes=[start];parents=[-1];found=None
    for iteration in range(4000):
        target=goal if rng.random()<.2 else rng.uniform(lo,hi);dist=np.linalg.norm(np.array(nodes)-target,axis=1);near=int(np.argmin(dist))
        if dist[near]<1e-12:continue
        q=nodes[near]+min(.18,dist[near])*(target-nodes[near])/dist[near]
        if not scene.edge(nodes[near],q):continue
        nodes.append(q);parents.append(near)
        if np.linalg.norm(q-goal)<.25 and scene.edge(q,goal):
            result=[goal];idx=len(nodes)-1
            while idx>=0:result.append(nodes[idx]);idx=parents[idx]
            found=np.array(result[::-1]);break
    elapsed=time.perf_counter()-begin
    if found is None:raise RuntimeError('RRT iteration budget exhausted')
    raw=found.copy()
    for _ in range(100):
        a,b=sorted(rng.choice(len(found),2,replace=False))
        if b>a+1 and scene.edge(found[a],found[b],spacing=.01):found=np.r_[found[:a+1],found[b:]]
    assert all(scene.edge(a,b,.005) for a,b in zip(found[:-1],found[1:]))
    return raw,found,{'seed':seed,'planning_seconds':elapsed,'iterations':iteration+1,'raw_length_rad':float(np.linalg.norm(np.diff(raw,axis=0),axis=1).sum()),'smoothed_length_rad':float(np.linalg.norm(np.diff(found,axis=0),axis=1).sum()),'direct_edge_valid':scene.edge(start,goal,.005),'fine_edge_spacing_rad':.005,'planning_clearance_margin_m':.015}
