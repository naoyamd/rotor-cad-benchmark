"""Run with FreeCAD's bundled Python or exec in FreeCAD. No external CAD assets."""
import csv
import json
import sys
from pathlib import Path
import FreeCAD as A
import Part
import numpy as np
from mechanism import P,OUT,solve,swash,rz,rx,ry,deg

STEEL=(.67,.72,.79); AL=(.77,.82,.86); BLUE=(.16,.36,.64)
GOLD=(.85,.62,.19); RUBBER=(.16,.18,.21); BLADE=(.25,.31,.37)
V=lambda p:A.Vector(*[float(x) for x in p])

def cyl(r,a,b):
    a,b=V(a),V(b); d=b-a
    return Part.makeCylinder(r,d.Length,a,d)
def ring(ro,ri,h,z=0):
    return Part.makeCylinder(ro,h,V([0,0,z])).cut(Part.makeCylinder(ri,h,V([0,0,z])))
def matrix(q,t=(0,0,0)):
    m=A.Matrix()
    for i in range(3):
        for j in range(3): setattr(m,'A%d%d'%(i+1,j+1),float(q[i,j]))
    m.A14,m.A24,m.A34=map(float,t)
    return m
def moved(s,q,t=(0,0,0)):
    result=s.copy()
    result.transformShape(matrix(q,t))
    return result

def foil(p):
    wires=[]
    # A faceted closed NACA0012 section, preserving a valid manufacturing envelope.
    x=(1-np.cos(np.linspace(0,np.pi,25)))/2
    yt=5*.12*(.2969*np.sqrt(x)-.126*x-.3516*x*x+.2843*x**3-.1036*x**4)*p['chord']
    for radius in [p['root'],.4*p['radius'],.75*p['radius'],p['radius']]:
        tw=deg(-8)*(radius-.75*p['radius'])/(p['radius']-p['root'])
        pts=[]
        for xx,zz in list(zip(x,yt))+list(zip(x[-2:0:-1],-yt[-2:0:-1])):
            # Leading edge +Y; quarter chord is the pitch axis.
            yz=rx(tw)@np.array([0,(.25-xx)*p['chord'],zz])
            pts.append(V([radius-p['fold_pin'],yz[1],yz[2]]))
        pts.append(pts[0]); wires.append(Part.makePolygon(pts))
    return Part.makeLoft(wires,True,False)

def build(name='Rotor_C1',collective=12,cc=0,cs=0,psi=0,fold=0,p=P,parked=False,flap=None,lag=None):
    if not all(np.isfinite(v) and v>0 for v in p.values()):raise ValueError('Design parameters must be finite and positive')
    if not p['radius']>p['root']>p['fold_pin']>p['joint']:raise ValueError('Invalid radial interfaces')
    if not 0<=fold<=1:raise ValueError('Fold fraction must be between zero and one')
    if not all(np.isfinite(v) for v in [collective,cc,cs,psi]):raise ValueError('Non-finite control input')
    flap=[0.]*5 if flap is None else list(flap)
    lag=[0.]*5 if lag is None else list(lag)
    if len(flap)!=5 or len(lag)!=5 or not all(np.isfinite(v) for v in flap+lag):raise ValueError('Five finite flap/lag angles required')
    if any(not -3<=v<=6 for v in flap) or any(abs(v)>3 for v in lag):raise ValueError('Outside declared flap/lag envelope')
    if (fold or parked) and any(flap+lag):raise ValueError('Park the articulated support before folding')
    doc=A.newDocument(name)
    params=doc.addObject('App::FeaturePython','Parameters')
    params.Label='C1 parameters / regenerate with build.py'
    for key,val in p.items():
        params.addProperty('App::PropertyFloat',key,'Design'); setattr(params,key,val)
    for key,val in dict(Collective=collective,CyclicCos=cc,CyclicSin=cs,Azimuth=psi,FoldFraction=fold).items():
        params.addProperty('App::PropertyFloat',key,'State'); setattr(params,key,val)
    params.addProperty('App::PropertyString','Status','Evidence'); params.Status='G1 provisional; not flight qualified'
    params.addProperty('App::PropertyBool','Parked','State');params.Parked=parked
    params.addProperty('App::PropertyFloatList','FlapAngles','State');params.FlapAngles=flap
    params.addProperty('App::PropertyFloatList','LagAngles','State');params.LagAngles=lag
    groups={k:doc.addObject('App::DocumentObjectGroup',k) for k in ['Fixed','Swashplate','Hub','Reference']}
    for i in range(5): groups['Blade%d'%(i+1)]=doc.addObject('App::DocumentObjectGroup','Blade%d'%(i+1))
    bom=[]
    def add(name,shape,group,color=STEEL,material='Steel, grade TBD',connection='',note=''):
        if not shape.isValid() or shape.Volume<=0: raise ValueError('Invalid solid: '+name)
        ob=doc.addObject('PartDesign::Feature',name); ob.Shape=shape
        groups[group].addObject(ob)
        for key,val in [('MaterialBasis',material),('Connection',connection),('EngineeringNote',note)]:
            ob.addProperty('App::PropertyString',key,'Engineering'); setattr(ob,key,val)
        if A.GuiUp:
            ob.ViewObject.ShapeColor=color; ob.ViewObject.LineColor=(.10,.13,.18)
        bom.append([name,1,material,group,connection,note or 'Capacity not verified'])
        return ob
    # Nonrotating sleeve surrounds a free rotating mast; two interfaces are distinct.
    add('Mast',ring(160,115,2440),'Hub',connection='gearbox / hub')
    add('MountFlange',ring(300,115,90,-90),'Hub',connection='gearbox interface')
    add('FixedGuide',ring(190,170,1100,100),'Fixed',connection='airframe / sliding sphere')
    center,q=swash(collective,cc,cs,p)
    if fold or parked:
        # Zero pitch parking setup keeps the same fixed-length links connected.
        collective=float(np.rad2deg(np.arcsin((p['hub_z']-p['link_length']-p['swash_z']-p['swash_joint_z'])/p['horn']+np.sin(deg(12)))))
        params.Collective=collective
        center,q=swash(collective,0,0,p)
    slider=ring(225,192,280,-140)
    add('SlidingSleeve',moved(slider,np.eye(3),center),'Swashplate',connection='FixedGuide axial slide')
    sphere=Part.makeSphere(280).cut(Part.makeCylinder(225,700,V([0,0,-350])))
    add('SupportSphere',moved(sphere,np.eye(3),center),'Swashplate',connection='SlidingSleeve / socket')
    socket=Part.makeSphere(310).cut(Part.makeSphere(282)).common(Part.makeBox(800,800,280,V([-400,-400,-140])))
    add('TiltSocket',moved(socket,q,center),'Swashplate',connection='SupportSphere spherical tilt',note='2 mm visualization clearance; bearing fit not toleranced')
    stator=ring(740,570,55,-100)
    for angle in [0,120,240]:
        arm=Part.makeBox(310,85,55,V([290,-42.5,-100]))
        stator=stator.fuse(moved(arm,rz(deg(angle))))
    add('Stator',moved(stator,q,center),'Swashplate',BLUE,'Al alloy TBD','TiltSocket / lower bearing race')
    rotor=ring(740,570,55,-5)
    for j in range(5):
        axis=rz(deg(psi+j*72))@np.array([p['horn_x'],p['horn'],0.])
        rotor=rotor.fuse(cyl(32,axis+[0,0,40],axis+[0,0,p['swash_joint_z']]))
    add('RotorRing',moved(rotor,q,center),'Swashplate',BLUE,'Al alloy TBD','upper bearing race / 5 raised ball lugs')
    groove=Part.makeTorus(652,6,V([0,0,-25]))
    add('LowerRace',moved(ring(700,605,15,-45).cut(groove),q,center),'Swashplate',connection='Stator / bearing rollers')
    add('UpperRace',moved(ring(700,605,15,-20).cut(groove),q,center),'Swashplate',connection='RotorRing / bearing rollers')
    # Discrete rolling elements document the rotating interface, not a rated bearing design.
    balls=[]
    for j in range(36): balls.append(Part.makeSphere(6,V([652*np.cos(j*np.pi/18),652*np.sin(j*np.pi/18),-25])))
    add('BearingRollingElements',moved(Part.makeCompound(balls),q,center),'Swashplate',GOLD,connection='LowerRace / UpperRace',note='Concept rolling race; contact stress and rated capacity unresolved')
    for j,angle in enumerate([30,150,270]):
        er=np.array([np.cos(deg(angle)),np.sin(deg(angle)),0.])
        bottom=er*710+[0,0,150]; top=center+q@(er*710+[0,0,-100])
        d=top-bottom; d/=np.linalg.norm(d)
        add('ServoBody%d'%j,cyl(55,bottom,bottom+550*d).cut(cyl(27,bottom+40*d,bottom+551*d)),'Fixed',BLUE,connection='airframe / piston')
        add('ServoRod%d'%j,cyl(25,top-650*d,top),'Fixed',connection='piston / Stator',note='650 mm rigid rod slides inside body; capacity unverified')
        for suffix,pt in [('Lower',bottom),('Upper',top)]: add('Servo%d%sJoint'%(j,suffix),Part.makeSphere(40,V(pt)),'Fixed',GOLD,connection='spherical joint')
    def scissors(name,a,b,group,length=520):
        d=b-a; dist=np.linalg.norm(d)
        if dist>=2*length: raise ValueError('Scissor cannot close')
        out=np.array([a[0],a[1],0.]); out/=np.linalg.norm(out)
        out-=d*np.dot(out,d)/dist**2; out/=np.linalg.norm(out)
        elbow=(a+b)/2+out*np.sqrt(length**2-dist**2/4)
        for suffix,start,end in [('A',a,elbow),('B',elbow,b)]:
            add(name+suffix,cyl(22,start,end),group,connection='universal end / revolute elbow',note='Centre-distance closure only; joint clocking not fully verified')
        for j,pt in enumerate([a,elbow,b]):add(name+'Joint%d'%j,Part.makeSphere(35,V(pt)),group,GOLD,connection='scissor pivot')
    scissors('Antirotation',np.array([900.,0,350]),center+q@np.array([740.,0,-80]),'Fixed',550)
    az=rz(deg(psi))
    scissors('RotatingDrive',az@np.array([350.,0,1900]),center+q@az@np.array([740.,0,35]),'Hub',510)
    add('HubRing',ring(380,160,210,p['hub_z']-105),'Hub',connection='Mast / five bearing housings')
    add('HubCap',ring(400,160,45,p['hub_z']+105),'Hub',connection='hub retention')
    blade_shape=foil(p)
    fold_angles=[0,-72,36,-36,72]
    for i in range(5):
        g='Blade%d'%(i+1); angle=psi+i*72; az=rz(deg(angle)); s=solve(angle,collective,cc if not (fold or parked) else 0,cs if not (fold or parked) else 0,flap[i],lag[i],p=p)
        rot=s['rotation']; pivot=s['pivot']
        def loc(pt):return pivot+rot@np.array(pt,dtype=float)
        def ls(shape):return moved(shape,rot,pivot)
        fixed=lambda pt:az@np.array(pt,dtype=float)+[0,0,p['hub_z']]
        fork=Part.makeCompound([cyl(35,fixed([320,y,0]),fixed([480,y,0])) for y in [-135,135]])
        fork=fork.cut(Part.makeSphere(175,V(pivot)))
        add(g+'HubArm',fork,'Hub',connection=g+'OuterRace / HubRing')
        radial_sphere=lambda radius:Part.makeSphere(radius,V([0,0,0]),V([1,0,0]))
        outer=radial_sphere(190).cut(radial_sphere(175)).cut(cyl(95,[-250,0,0],[250,0,0]))
        add(g+'OuterRace',moved(outer,az,pivot),g,connection='HubArm / laminated bearing')
        elast=[]; shims=[]
        for k in range(5):
            r=146+k*5.8
            el=radial_sphere(r+4.5).cut(radial_sphere(r)).cut(cyl(90,[-250,0,0],[250,0,0]))
            sh=radial_sphere(r+5.8).cut(radial_sphere(r+4.5)).cut(cyl(90,[-250,0,0],[250,0,0]))
            elast.append(el); shims.append(sh)
        add(g+'Elastomer',moved(Part.makeCompound(elast),az,pivot),g,RUBBER,'Elastomer TBD','InnerRace / shims',note='Undeformed layer envelope; finite stiffness assigned in basis, capacity unproven')
        add(g+'Shims',moved(Part.makeCompound(shims),az,pivot),g,connection='laminated bearing')
        add(g+'InnerRace',ls(Part.makeSphere(145)),g,connection='Cuff / elastomer')
        add(g+'Cuff',ls(cyl(65,[0,0,0],[p['fold_pin']-p['joint']-320,0,0])),g,connection='InnerRace / fold clevis')
        horn=cyl(28,[240,0,0],[240,250,0]).fuse(cyl(28,[240,250,0],[0,250,0]))
        add(g+'PitchHorn',ls(horn),g,connection='Cuff / pitch link')
        link=add(g+'PitchLink',cyl(17,s['bottom'],s['top']),g,connection='RotorRing / PitchHorn',note='Fixed length; finite-angle closure solved')
        for suffix,pt in [('LowerBall',s['bottom']),('UpperBall',s['top'])]:add(g+suffix,Part.makeSphere(30,V(pt)),g,GOLD,connection='spherical rod end')
        for prop,value in [('BottomCentre',s['bottom']),('TopCentre',s['top'])]:link.addProperty('App::PropertyVector',prop,'Kinematics');setattr(link,prop,V(value))
        link.addProperty('App::PropertyAngle','SolvedPitch','Kinematics');link.SolvedPitch=s['theta']
        # In-plane hydraulic lag damper and explicit brackets.
        da=fixed([420,-400,0]); db=loc([450,-350,0]); dr=db-da; dr/=np.linalg.norm(dr)
        add(g+'DamperMount',cyl(35,fixed([360,-140,0]),da),g,connection='Hub / damper fixed ball')
        add(g+'DamperLug',cyl(30,loc([450,0,0]),db),g,connection='Cuff / damper moving ball')
        add(g+'DamperBody',cyl(48,da,da+320*dr).cut(cyl(22,da+40*dr,da+321*dr)),g,BLUE,connection='fixed bracket / piston')
        add(g+'DamperRod',cyl(20,db-450*dr,db),g,connection='piston / moving bracket')
        for suffix,pt in [('DA',da),('DB',db)]:add(g+suffix,Part.makeSphere(40,V(pt)),g,GOLD,connection='damper spherical end')
        # Double-shear clevis, vertical hinge and off-axis removable flight lock.
        fp=p['fold_pin']-p['joint']; plates=[]
        for z in [-130,90]:
            plate=Part.makeBox(520,300,40,V([fp-420,-150,z]))
            for x in [fp,fp-220]:plate=plate.cut(cyl(43,[x,0,z-1],[x,0,z+41]))
            plates.append(plate)
        bridge=Part.makeBox(100,300,260,V([fp-420,-150,-130]))
        add(g+'FoldClevis',ls(Part.makeCompound(plates+[bridge])),g,connection='Cuff / hinge and lock')
        foldq=rz(deg(fold_angles[i]*fold)); foldorigin=loc([fp,0,0]); fq=rot@foldq
        tang=Part.makeBox(430,220,160,V([-270,-110,-80]))
        for x in [0,-220]:tang=tang.cut(cyl(43,[x,0,-81],[x,0,81]))
        add(g+'FoldTang',moved(tang,fq,foldorigin),g,connection='blade root / fold hinge')
        add(g+'FoldHinge',ls(cyl(42.5,[fp,0,-155],[fp,0,155])),g,GOLD,connection='clevis / tang')
        # The withdrawn flight pin remains above its bore, not removed from the model.
        withdrawal=320 if fold or parked else 0
        add(g+'FlightLock',ls(cyl(42.5,[fp-220,0,-155+withdrawal],[fp-220,0,155+withdrawal])),g,GOLD,connection='offset lock / tethered parking position',note='Withdrawal precedes fold; retaining cap/interlock detail provisional')
        blade=add(g+'Airfoil',moved(blade_shape,fq,foldorigin),g,BLADE,'Composite equivalent','FoldTang / spar',note='Full length NACA0012 envelope; internal laminate not designed')
        blade.addProperty('App::PropertyMass','EquivalentMass','Mass');blade.EquivalentMass=p['blade_mass']
        blade.addProperty('App::PropertyVector','EquivalentCG','Mass');blade.EquivalentCG=V(foldorigin+fq@np.array([(p['root']+p['radius'])/2-p['fold_pin'],0,0]))
        # Root envelope joins the aerodynamic section to the tang; not a laminate model.
        spar=Part.makeBox(270,160,100,V([-50,-80,-50])).cut(cyl(43,[0,0,-51],[0,0,51]))
        add(g+'RootSparEnvelope',moved(spar,fq,foldorigin),g,BLADE,'Composite equivalent','FoldTang / Airfoil')
    fus=add('AirframeEnvelope',Part.makeBox(13000,3600,3000,V([-6500,-1800,-3000])),'Reference',(.5,.6,.7),'Boundary only','fixed airframe',note='Not an actual fuselage design')
    if A.GuiUp: fus.ViewObject.Visibility=False
    doc.recompute()
    if A.GuiUp:
        import FreeCADGui as G
        G.activeDocument().activeView().viewAxonometric();G.activeDocument().activeView().fitAll()
    return doc,bom

def components(doc):return [o for o in doc.Objects if o.TypeId=='PartDesign::Feature' and o.Name!='AirframeEnvelope']

def save(doc,bom,stem):
    doc.recompute(); doc.saveAs(str(OUT/(stem+'.FCStd')))
    Part.export(components(doc),str(OUT/(stem+'.step')))
    with (OUT/'bom.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.writer(f);w.writerow(['CAD_name','quantity','material','assembly','connection','evaluation']);w.writerows(bom)

if __name__=='__main__':
    d,b=build();save(d,b,'rotor-neutral')
    d2,b2=build('Rotor_MaxInput',16,3,0,36);save(d2,b2,'rotor-max-input')
    d3,b3=build('Rotor_Folded',fold=1);save(d3,b3,'rotor-folded')
    d4,b4=build('Rotor_Parked',parked=True);save(d4,b4,'rotor-parked')
    print('CAD_BUILD_COMPLETE',len(components(d)),flush=True)
