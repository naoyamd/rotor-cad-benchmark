"""Millimetre BREP primitives. No visual clearances are added to bearing fits."""
import FreeCAD as A
import Part
import numpy as np

V=lambda p:A.Vector(*map(float,p))
def unit(v):
    v=np.asarray(v,dtype=float); n=np.linalg.norm(v)
    if n<1e-10: raise ValueError('Zero direction')
    return v/n
def cyl(r,a,b):
    a,b=V(a),V(b); d=b-a
    return Part.makeCylinder(r,d.Length,a,d)
def ring(ro,ri,h,z=0):
    return cyl(ro,[0,0,z],[0,0,z+h]).cut(cyl(ri,[0,0,z-1],[0,0,z+h+1]))
def moved(shape,q,t=(0,0,0)):
    m=A.Matrix()
    for i in range(3):
        for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),float(q[i,j]))
    m.A14,m.A24,m.A34=map(float,t)
    s=shape.copy();s.transformShape(m);return s
def axis_frame(axis,radial=None):
    z=unit(axis)
    v=np.array([1.,0,0]) if abs(z[0])<.85 else np.array([0.,1,0])
    if radial is not None:v=np.asarray(radial)
    x=unit(v-z*np.dot(z,v));y=np.cross(z,x)
    return np.column_stack([x,y,z])
def fuse(shapes):
    return shapes[0].multiFuse(shapes[1:]).removeSplitter() if len(shapes)>1 else shapes[0]
def eye(radius,bore,thick,center,axis):
    axis=unit(axis);center=np.array(center)
    return cyl(radius,center-axis*thick/2,center+axis*thick/2).cut(cyl(bore,center-axis*(thick/2+1),center+axis*(thick/2+1)))
def foil(p):
    from mechanism import rx,deg
    x=(1-np.cos(np.linspace(0,np.pi,25)))/2
    yt=5*.12*(.2969*np.sqrt(x)-.126*x-.3516*x*x+.2843*x**3-.1036*x**4)*p['chord']
    wires=[]
    for radius in [p['root'],.4*p['radius'],.75*p['radius'],p['radius']]:
        tw=deg(-8)*(radius-.75*p['radius'])/(p['radius']-p['root'])
        pts=[]
        for xx,zz in list(zip(x,yt))+list(zip(x[-2:0:-1],-yt[-2:0:-1])):
            yz=rx(tw)@np.array([0,(.25-xx)*p['chord'],zz]);pts.append(V([radius-p['fold_pin'],yz[1],yz[2]]))
        wires.append(Part.makePolygon(pts+[pts[0]]))
    return Part.makeLoft(wires,True,False)
