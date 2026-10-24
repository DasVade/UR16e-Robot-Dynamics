"""Small mathematical/frame checks and independent saved-metric recomputation."""
import unittest,json,os
import numpy as np
from urdf_model import Model,ROOT
class Verification(unittest.TestCase):
    def test_recorded_independent_checks(self):
        d=json.loads((ROOT/'docs/executed/half_step_verification.json').read_text())
        self.assertGreater(d['minimum_mass_eigenvalue'],1e-6)
        for name,limit in [('mass_reference',1e-10),('gravity_reference',1e-6),('bias_reference',1e-5),('jacobian_fd',1e-7),('external_wrench_acceleration',1e-5)]:self.assertLess(d[name],limit)
    def test_reconciled_dh_inertias(self):
        model=Model();params=json.loads((ROOT/'docs/executed/reconciled_parameters.json').read_text())
        for q in np.random.default_rng(5170).uniform(-1,1,(5,6)):
            T=np.eye(4);points=[];axes=[];M=np.zeros((6,6))
            for k,(a,d,al,item) in enumerate(zip([0,-.4784,-.36,0,0,0],[.1807,0,0,.17415,.11985,.11655],[np.pi/2,0,0,np.pi/2,-np.pi/2,0],params)):
                points.append(T[:3,3].copy());axes.append(T[:3,2].copy());c,s=np.cos(q[k]),np.sin(q[k]);ca,sa=np.cos(al),np.sin(al);T=T@np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]])
                p=T[:3,:3]@item['com_DH_m']+T[:3,3];J=np.zeros((6,6))
                for j in range(k+1):J[:,j]=np.r_[np.cross(axes[j],p-points[j]),axes[j]]
                I=T[:3,:3]@np.array(item['inertia_DH'])@T[:3,:3].T;M+=item['mass']*J[:3].T@J[:3]+J[3:].T@I@J[3:]
            self.assertLess(np.max(abs(M-model.mass_potential(q)[0])),1e-8)
    def test_numerical_jacobian(self):
        from scipy.spatial.transform import Rotation
        m=Model();q=np.array([.3,-1,.6,.2,.9,.1]);J=m.jac(q)
        for k in range(6):
            e=np.eye(6)[k]*1e-6;A=m.frames(q+e)[0]['tool0'];B=m.frames(q-e)[0]['tool0'];fd=np.r_[(A[:3,3]-B[:3,3])/2e-6,Rotation.from_matrix(A[:3,:3]@B[:3,:3].T).as_rotvec()/2e-6]
            self.assertLess(np.max(abs(fd-J[:,k])),1e-8)
    def test_saved_tracking_metrics(self):
        path=ROOT/os.environ.get('UR16E_RUN','outputs/dynamics_20261024_02');metrics=json.loads((path/'tracking_metrics.json').read_text());limit=np.array([330,330,150,56,56,56])
        for row in metrics:
            d=np.load(path/(row['case']+'.npz'));err=d['q']-d['desired'];np.testing.assert_allclose(np.sqrt(np.mean(err**2,axis=0)),row['rmse_rad'],rtol=1e-12)
            np.testing.assert_allclose(abs(d['applied']).max(axis=0),row['peak_applied_Nm'],rtol=1e-12);self.assertTrue((abs(d['applied'])<=limit+1e-10).all())
if __name__=='__main__':unittest.main()
