"""URDF model plus independent Jacobian/energy dynamics reference (SI units)."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
JOINTS=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
def vec(s,default='0 0 0'):return np.fromstring(s or default,sep=' ')
def origin(e):
    T=np.eye(4)
    if e is not None:T[:3,:3]=Rotation.from_euler('xyz',vec(e.get('rpy'))).as_matrix();T[:3,3]=vec(e.get('xyz'))
    return T
class Model:
    def __init__(self,path=ROOT/'simulink/UR16e.urdf'):
        self.xml=ET.parse(path);self.links={e.get('name'):e for e in self.xml.findall('link')};self.joints=self.xml.findall('joint')
    def frames(self,q):
        frames={'base_link':np.eye(4)};axes={};anc={'base_link':[]};pending=list(self.joints)
        while pending:
            for j in pending[:]:
                parent=j.find('parent').get('link');child=j.find('child').get('link');name=j.get('name')
                if parent not in frames:continue
                T=frames[parent]@origin(j.find('origin'));chain=list(anc[parent])
                if name in JOINTS:
                    k=JOINTS.index(name);axis=vec(j.find('axis').get('xyz'));axes[k]=(T[:3,3].copy(),T[:3,:3]@axis)
                    R=np.eye(4);R[:3,:3]=Rotation.from_rotvec(axis*q[k]).as_matrix();T=T@R;chain.append(k)
                frames[child]=T;anc[child]=chain;pending.remove(j)
        return frames,axes,anc
    def jac(self,q,link='tool0',local=np.zeros(3)):
        f,axes,anc=self.frames(q);p=f[link][:3,:3]@local+f[link][:3,3];J=np.zeros((6,6))
        for k in anc[link]:o,z=axes[k];J[:,k]=np.r_[np.cross(z,p-o),z]
        return J
    def mass_potential(self,q):
        f,axes,anc=self.frames(q);M=np.zeros((6,6));U=0.
        for name,e in self.links.items():
            inert=e.find('inertial')
            if inert is None or not anc[name]:continue
            mass=float(inert.find('mass').get('value'));C=f[name]@origin(inert.find('origin'));p=C[:3,3]
            a=inert.find('inertia').attrib;I=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
            J=np.zeros((6,6))
            for k in anc[name]:o,z=axes[k];J[:,k]=np.r_[np.cross(z,p-o),z]
            M+=mass*J[:3].T@J[:3]+J[3:].T@C[:3,:3]@I@C[:3,:3].T@J[3:];U+=mass*9.81*p[2]
        return M,U
    def dynamics(self,q,v):
        h=2e-5;dM=np.zeros((6,6,6));g=np.zeros(6)
        for k in range(6):
            e=np.eye(6)[k]*h;Mp,Up=self.mass_potential(q+e);Mm,Um=self.mass_potential(q-e);dM[:,:,k]=(Mp-Mm)/(2*h);g[k]=(Up-Um)/(2*h)
        c=np.zeros(6)
        for i in range(6):
            for j in range(6):
                for k in range(6):c[i]+=.5*(dM[i,j,k]+dM[i,k,j]-dM[j,k,i])*v[j]*v[k]
        return self.mass_potential(q)[0],c,g
def prepare(output):
    """Resolve package paths; convert existing DAE geometry to OBJ for Bullet."""
    import trimesh
    output=Path(output);output.mkdir(parents=True,exist_ok=True);m=Model()
    for mesh in m.xml.findall('.//mesh'):
        rel=mesh.get('filename').split('package://ur_description/')[-1];src=ROOT/'simulink/model'/rel
        assert src.exists(),src
        if src.suffix=='.dae':
            dst=output/(src.parent.parent.name+'_'+src.stem+'.obj')
            if not dst.exists():
                scene=trimesh.load(src,force='scene');combined=scene.to_mesh();combined.export(dst)
            mesh.set('filename',str(dst.resolve()))
        else:mesh.set('filename',str(src.resolve()))
    # Massless fixed helper frames get explicit zero inertia to prevent Bullet defaults.
    for link in m.xml.findall('link'):
        if link.find('inertial') is None:
            inert=ET.SubElement(link,'inertial');ET.SubElement(inert,'mass',value='0');ET.SubElement(inert,'inertia',ixx='0',iyy='0',izz='0',ixy='0',ixz='0',iyz='0')
    path=output/'resolved.urdf';m.xml.write(path);return path
