"""Shared, declared controller configurations; gains are not equivalent units."""
import numpy as np
PD_KP=np.array([180,180,120,50,30,20.]);PD_KD=np.array([30,35,22,8,5,3.])
CT_KP=np.full(6,64.);CT_KD=np.full(6,16.)
def command(method,q,v,qd,vd,ad,inverse,gravity):
    if method=='pd_gravity':return PD_KP*(qd-q)+PD_KD*(vd-v)+gravity(q)
    if method=='computed_torque':return inverse(q,v,ad+CT_KP*(qd-q)+CT_KD*(vd-v))
    raise ValueError(method)
