import unittest,json
import numpy as np
import pybullet as p
from robot import Robot,reference
from planner import CollisionScene
from urdf_model import ROOT
class MigrationTests(unittest.TestCase):
    def tearDown(self):
        if p.isConnected():p.disconnect()
    def test_complete_robot_api(self):
        r=Robot(ROOT/'outputs/test_model');q=np.array([.3,-1,.7,-.4,.9,.1]);v=np.full(6,.1);a=np.full(6,.2);w=np.array([1,2,3,.1,.2,.3]);T=r.fk(q);z,check=r.ik(T,q+.1)
        self.assertLess(check['position_residual_m'],1e-8);self.assertLess(check['orientation_residual_rad'],1e-8)
        np.testing.assert_allclose(r.forward(q,v,r.inverse(q,v,a,w),w),a,atol=1e-9)
        M,c,g=r.model.dynamics(q,v);np.testing.assert_allclose(r.mass(q),M,atol=1e-9);np.testing.assert_allclose(r.gravity(q),g,atol=1e-6);np.testing.assert_allclose(r.velocity_terms(q,v),c,atol=1e-6)
        r.reset(q);state=p.getLinkState(r.body,r.tool,computeForwardKinematics=True);np.testing.assert_allclose(state[4],T[:3,3],atol=1e-6)
    def test_edge_preserving_time_parameterization(self):
        path=np.array([[0,0,0,0,0,0],[.1,.2,0,0,0,0],[.2,0,0,0,0,0]])
        t,q,v,a=reference(path,.001);self.assertTrue(np.all(np.diff(t)>0));np.testing.assert_allclose(q[0],path[0]);np.testing.assert_allclose(q[-1],path[-1]);np.testing.assert_allclose(v[[0,-1]],0);np.testing.assert_allclose(a[[0,-1]],0)
        for x in q:
            self.assertTrue(any(np.linalg.norm(x-(u+np.clip(np.dot(x-u,z-u)/np.dot(z-u,z-u),0,1)*(z-u)))<1e-10 for u,z in zip(path[:-1],path[1:])))
    def test_curated_dynamic_validation(self):
        d=json.loads((ROOT/'docs/migration/metrics.json').read_text());self.assertEqual(d['actuator_limit_violations'],0);self.assertEqual(d['actual_velocity_limit_violations'],0)
        for name in ['reference','actual']:
            self.assertGreater(d['validation'][name]['minimum_sampled_clearance_m'],0)
            self.assertEqual(d['validation'][name]['colliding_samples'],0);self.assertEqual(d['validation'][name]['invalid_dense_edges'],0);self.assertEqual(d['validation'][name]['position_limit_violations'],0)
        scene=CollisionScene(ROOT/'outputs/test_model');path=np.array(json.loads((ROOT/'docs/migration/path.json').read_text())['smoothed'])
        self.assertFalse(scene.edge(path[0],path[-1],.005));self.assertTrue(all(scene.edge(a,b,.005) for a,b in zip(path[:-1],path[1:])))
if __name__=='__main__':unittest.main()
