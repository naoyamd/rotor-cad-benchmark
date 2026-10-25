"""C2 rotor: explicit bearing eyes, double-shear pins and constrained scissors.

Run with FreeCAD's Python. Geometry is generated from solved joint centres;
the solver, fit dimensions and the saved state travel with the native model.
"""
import csv,json
from pathlib import Path
import FreeCAD as A
import Part
import numpy as np
from mechanism import P,OUT,solve,swash_frames,rx,ry,rz,deg
from geometry import V,unit,cyl,ring,moved,axis_frame,fuse,eye,foil

STEEL=(.66,.72,.79); BLUE=(.12,.33,.60); GOLD=(.89,.65,.23)
RUBBER=(.15,.17,.19); BLADE=(.28,.36,.43)

def build(name='Rotor_C2',collective=12.,cc=0.,cs=0.,psi=0.,fold=0.,p=P,parked=False,flap=None,lag=None):
    if not all(np.isfinite(v) for v in [collective,cc,cs,psi,fold]):raise ValueError('Nonfinite input')
    if not 0<=fold<=1:raise ValueError('Invalid fold fraction')
    if not p['radius']>p['root']>p['fold_pin']>p['joint']:raise ValueError('Invalid radial dimensions')
    flap=np.zeros(5) if flap is None else np.asarray(flap,dtype=float)
    lag=np.zeros(5) if lag is None else np.asarray(lag,dtype=float)
    if flap.shape!=(5,) or lag.shape!=(5,) or not np.all(np.isfinite([flap,lag])):raise ValueError('Five finite angles required')
    if np.any((flap< -3)|(flap>6)) or np.any(abs(lag)>3):raise ValueError('Articulation outside envelope')
    if parked or fold:
        if np.any(flap) or np.any(lag) or psi%360:raise ValueError('Folding requires indexed and level hub')
        collective=float(np.rad2deg(np.arcsin((p['hub_z']-p['link_length']-p['swash_z']-p['swash_joint_z'])/p['horn']+np.sin(deg(p['collective_ref'])))))
        cc=cs=0.
    elif not 8<=collective<=16 or np.hypot(cc,cs)>3.000001:raise ValueError('Control outside envelope')
    d=A.newDocument(name);bom=[];joints=[]
    params=d.addObject('App::FeaturePython','Parameters')
    params.addProperty('App::PropertyString','DesignJSON','Design');params.DesignJSON=json.dumps(p)
    for key,value in dict(Collective=collective,CyclicCos=cc,CyclicSin=cs,Azimuth=psi,FoldFraction=fold).items():
        params.addProperty('App::PropertyFloat',key,'State');setattr(params,key,value)
    for key,value in [('FlapAngles',flap),('LagAngles',lag)]:
        params.addProperty('App::PropertyFloatList',key,'State');setattr(params,key,list(value))
    params.addProperty('App::PropertyBool','Parked','State');params.Parked=bool(parked or fold)
    params.addProperty('App::PropertyString','DesignStatus','Evidence');params.DesignStatus='C2 initial-design candidate; acceptance in requirements.csv'
    groups={n:d.addObject('App::DocumentObjectGroup',n) for n in ['Fixed','Swashplate','Hub','Reference']+['Blade%d'%i for i in range(1,6)]}
    def add(n,shape,g,color=STEEL,connection='',note='',material='17-4 PH H1150 / screening basis'):
        if shape.isNull() or not shape.isValid() or shape.Volume<=0:raise ValueError('Invalid '+n)
        o=d.addObject('PartDesign::Feature',n);o.Shape=shape;groups[g].addObject(o)
        for prop,val in [('Connection',connection),('MaterialBasis',material),('EngineeringNote',note)]:
            o.addProperty('App::PropertyString',prop,'Engineering');setattr(o,prop,val)
        if A.GuiUp:o.ViewObject.ShapeColor=color;o.ViewObject.LineColor=(.12,.15,.19)
        bom.append([n,g,material,connection,note,len(shape.Solids)])
        return o
    def pin(n,c,axis,g,r=16,half=66):
        c=np.array(c);u=unit(axis)
        add(n,cyl(r-.1,c-u*half,c+u*half),g,GOLD,connection='double shear journal')
        add(n+'Head',cyl(r*1.5,c-u*(half+10),c-u*half),g,GOLD,connection=n,note='Integral headed pin')
        nut=eye(r*1.5,r,12,c+u*(half+6),u)
        add(n+'Retainer',nut,g,GOLD,connection=n,note='Thread envelope; castellated lock and cotter specified in assembly procedure')
    def spherical(n,c,axis,line,bracketdir,g,owner,reach=75):
        c=np.asarray(c);u=unit(axis);v=unit(line);w=unit(bracketdir)
        # Ball bore stays on the supporting pin; the eye follows its rigid rod.
        outer_axis=unit(u-v*np.dot(u,v))
        inner_local=Part.makeSphere(32).common(cyl(40,[0,0,-24],[0,0,24])).cut(cyl(16,[0,0,-40],[0,0,40]))
        inner=moved(inner_local,axis_frame(u),c)
        outer_local=cyl(46,[0,0,-14],[0,0,14]).cut(Part.makeSphere(32.15))
        outer_local=outer_local.fuse(cyl(20,[40,0,0],[80,0,0])).removeSplitter()
        outer=moved(outer_local,axis_frame(outer_axis,v),c)
        add(n+'Ball',inner,g,GOLD,connection=n+'Pin / '+n+'Eye')
        add(n+'Eye',outer,g,connection=owner,note='Split race housing required for assembly; 0.15 mm nominal sphere radial fit')
        # Two support ears carry the through pin. Their bridge is away from the rod.
        ears=[]
        for sign in [-1,1]:
            ec=c+sign*48*u
            ear=fuse([cyl(34,ec-u*10,ec+u*10),cyl(25,ec,ec+w*reach)])
            ear=ear.common(cyl(10000,ec-u*10,ec+u*10))
            ear=ear.cut(cyl(16.25,ec-u*36,ec+u*36));ears.append(ear)
            add(n+('SpacerL' if sign<0 else 'SpacerR'),eye(22,16,14,c+sign*31*u,u),g,connection=n+'Ball / fork')
        ears.append(cyl(25,c+w*reach-u*48,c+w*reach+u*48))
        add(n+'Fork',fuse(ears),g,BLUE,connection=owner+' / '+n+'Pin')
        pin(n+'Pin',c,u,g)
        joints.append(dict(name=n,type='spherical pin/eye',center=list(map(float,c)),axis=list(map(float,u)),eye_axis=list(map(float,outer_axis)),misalignment_deg=float(np.rad2deg(np.arccos(np.clip(abs(u@outer_axis),-1,1))))))
        return c+w*reach
    def rrs(n,s,g,end_axis):
        a,e,b=s['base'],s['elbow'],s['end'];u=s['base_axis']
        # A: paired plates, B: central eye. The split prevents coincident solid arms.
        parts=[]
        for sign in [-1,1]:
            aa=a+sign*36*u;ee=e+sign*36*u
            part=fuse([cyl(21,aa,ee),eye(35,16.25,20,aa,u),eye(35,16.25,20,ee,u)])
            part=part.common(cyl(10000,aa-u*10,aa+u*10))
            part=part.cut(cyl(16.25,a-u*80,a+u*80)).cut(cyl(16.25,e-u*80,e+u*80));parts.append(part)
        add(n+'ArmA',Part.makeCompound(parts),g,BLUE,connection=n+'BasePin / '+n+'ElbowPin')
        second=fuse([eye(35,16.25,40,e,u),cyl(24,e+unit(b-e)*30,b-unit(b-e)*80)])
        second=second.cut(cyl(16.25,e-u*60,e+u*60))
        add(n+'ArmB',second,g,BLUE,connection=n+'ElbowPin / '+n+'EndEye')
        pin(n+'BasePin',a,u,g,half=60);pin(n+'ElbowPin',e,u,g,half=60)
        # Base journal belongs to the support, whose bracket is added below.
        add(n+'BaseBearing',eye(36,16.25,40,a,u),g,connection=n+'Support')
        spherical(n+'End',b,end_axis,e-b,-s['radial_basis'],g,n+'ArmB')
        joints.append(dict(name=n,type='RR-S',base=a.tolist(),elbow=e.tolist(),end=b.tolist(),axis=u.tolist(),lengths=s['arm_lengths_mm']))

    frames=swash_frames(collective,cc,cs,psi,p);center=frames['center'];qs=frames['stator_rotation'];qr=frames['rotor_rotation'];az=rz(deg(psi))
    # Airframe fixed carrier and mast form separate, continuous support paths.
    add('Mast',ring(160,115,2440),'Hub',connection='GearboxBoundary / Hub')
    flange=ring(300,115,90,-90)
    for j in range(12):
        xy=240*np.array([np.cos(j*np.pi/6),np.sin(j*np.pi/6)])
        flange=flange.cut(cyl(15,[*xy,-91],[*xy,1]))
    lockxy=260*np.array([np.cos(deg(15)),np.sin(deg(15)),0.])
    flange=flange.cut(cyl(20.5,lockxy+[0,0,-91],lockxy+[0,0,1]))
    add('MastFlange',flange,'Hub',connection='GearboxBoundary / Mast')
    indexer=[]
    for zz in [-120,20]:
        sh=Part.makeBox(150,90,25,V([220,-45,zz])).cut(cyl(20.5,[260,0,zz-1],[260,0,zz+26]))
        indexer.append(moved(sh,rz(deg(15))))
    indexer.append(moved(Part.makeBox(35,90,165,V([335,-45,-120])),rz(deg(15))))
    add('MastIndexBracket',fuse(indexer),'Fixed',BLUE,connection='AirframeCarrier / MastIndexPin')
    iz=0 if parked or fold else 180
    add('MastIndexPin',cyl(20,lockxy+[0,0,-125+iz],lockxy+[0,0,50+iz]),'Fixed',GOLD,connection='MastFlange / MastIndexBracket',note='Engaged only after zero-speed and azimuth index; trapped key releases fold-pin handles')
    add('MastIndexGuide',moved(ring(32,20.5,210,45),np.eye(3),lockxy),'Fixed',connection='MastIndexBracket / MastIndexPin')
    base=ring(980,320,70,-70)
    for angle in [30,150,270]:
        er=rz(deg(angle))[:,0];base=base.fuse(cyl(75,er*710+[0,0,0],er*710+[0,0,75]))
    add('AirframeCarrier',base,'Fixed',BLUE,connection='AirframeBoundary',note='Rigid interface; six-component reaction specification in design-structure.md')
    index_tool=cyl(32.25,lockxy+[0,0,-150],lockxy+[0,0,300])
    guide=fuse([ring(190,170,1270,10),ring(340,170,65,10),ring(380,320,100,-50)]).cut(index_tool)
    add('FixedGuide',guide,'Fixed',BLUE,connection='AirframeCarrier / sliding bushes')
    add('GuideBridge',ring(410,190,40,30).cut(index_tool),'Fixed',connection='FixedGuide / AirframeCarrier')
    # GuideBridge radial extension to the outer carrier.
    for j in range(3):
        bar=Part.makeBox(610,100,50,V([330,-50,-50]))
        add('CarrierSpoke%d'%j,moved(bar,rz(deg(j*120))),'Fixed',connection='AirframeCarrier / FixedGuide')
    sleeve=ring(225,202,300,-150)
    for z in [-145,100]:add('GuideBush'+str(z).replace('-','M'),moved(ring(202,190.15,45,z),np.eye(3),center),'Swashplate',GOLD,connection='FixedGuide / SlidingSleeve',material='Bronze/PTFE liner; qualification required')
    sphere=Part.makeSphere(280).cut(cyl(202,[0,0,-350],[0,0,350])).fuse(sleeve).removeSplitter()
    add('SliderSphere',moved(sphere,np.eye(3),center),'Swashplate',connection='GuideBushes / StatorSocket',note='One-piece bored spherical slider eliminates unsupported sphere-to-sleeve seam')
    socket=Part.makeSphere(310).cut(Part.makeSphere(280.15)).common(Part.makeBox(800,800,300,V([-400,-400,-150])))
    stator=ring(740,570,40,-120)
    for angle in [0,120,240]:stator=stator.fuse(moved(Part.makeBox(350,100,40,V([260,-50,-120])),rz(deg(angle))))
    for xx in [-320,260]:stator=stator.fuse(Part.makeBox(60,100,80,V([xx,-50,-120])))
    stator=stator.fuse(socket).cut(Part.makeSphere(280.15)).removeSplitter()
    stator=stator.cut(Part.makeBox(1600,.1,500,V([-800,-.05,-250])))
    split_bolts=[]
    for xx in [-290,290]:
        for zz in [-100,-60]:
            stator=stator.cut(cyl(6.5,[xx,-80,zz],[xx,80,zz]))
            split_bolts.append(cyl(6,[xx,-60,zz],[xx,60,zz]))
    add('StatorSocket',moved(stator,qs,center),'Swashplate',BLUE,connection='SliderSphere / LowerRace / servo forks',note='Split spherical socket belt captures slider at both apertures; machining split and flange bolts required')
    add('SocketSplitBolts',moved(Part.makeCompound(split_bolts),qs,center),'Swashplate',GOLD,connection='StatorSocket two halves',note='Four M12 through bolts; end/thread envelopes simplified')
    # Grooved opposed races; bolts and cage retain the bearing stack.
    groove=Part.makeTorus(652,15.10,V([0,0,-50]))
    lower=ring(651.9,590,60,-80).cut(groove)
    upper=ring(720,652.1,69.8,-79.8).cut(groove)
    upper=upper.cut(Part.makeBox(1600,1600,.05,V([-800,-800,-50.025])))
    add('LowerRace',moved(lower,qs,center),'Swashplate',connection='StatorSocket / rolling elements',note='Fixed INNER grooved race; custom bearing supplier load-rating gate')
    add('UpperRace',moved(upper,qr,center),'Swashplate',connection='RotorRing / rolling elements',note='Rotating OUTER race; radial split captures balls axially in both directions')
    balls=[Part.makeSphere(15,V([652*np.cos(j*np.pi/36),652*np.sin(j*np.pi/36),-50])) for j in range(72)]
    add('BearingBalls',moved(Part.makeCompound(balls),qs,center),'Swashplate',GOLD,connection='LowerRace / UpperRace')
    cage=Part.makeTorus(652,4,V([0,0,-50]))
    for ball in balls:cage=cage.cut(ball.makeOffsetShape(.6,.01))
    add('BearingCage',moved(cage,qs,center),'Swashplate',GOLD,connection='BearingBalls',material='Polymer separator pieces',note='Individual separators within race groove; not a rigid rotating cage')
    rotor=ring(740,570,35,-10)
    add('RotorRing',moved(rotor,qr,center),'Swashplate',BLUE,connection='UpperRace / five pitch forks')
    for which,rot,z,bolt_radius in [('Lower',qs,-120,610),('Upper',qr,-79.5,704)]:
        bolts=[]
        holes=[]
        bolt_length=55 if which=='Lower' else 109.5
        for j in range(18):
            pt=np.array([bolt_radius*np.cos(j*np.pi/9),bolt_radius*np.sin(j*np.pi/9),z])
            bolts.append(cyl(6,pt,pt+[0,0,bolt_length]))
            holes.append(cyl(6.5,pt-[0,0,10],pt+[0,0,bolt_length+10]))
        tool=moved(Part.makeCompound(holes),rot,center)
        if which=='Upper':upper_fastener_tool=tool
        for obname in ([which+'Race','StatorSocket'] if which=='Lower' else [which+'Race','RotorRing']):
            ob=d.getObject(obname);ob.Shape=ob.Shape.cut(tool)
        add(which+'RaceBolts',moved(Part.makeCompound(bolts),rot,center),'Swashplate',GOLD,connection=which+'Race / ring',note='M12 thread envelopes with 13 mm bores; preload specified by bearing supplier')
    for j,angle in enumerate([30,150,270]):
        er=rz(deg(angle))[:,0];tangent=rz(deg(angle))[:,1]
        bottom=er*710+[0,0,150];top=center+qs@(er*710+[0,0,-190]);line=unit(top-bottom)
        spherical('Servo%dLower'%j,bottom,tangent,line,-np.array([0,0,1.]),'Fixed','AirframeCarrier')
        spherical('Servo%dUpper'%j,top,qs@tangent,-line,qs[:,2],'Fixed','StatorSocket')
        body=cyl(55,bottom+line*80,bottom+line*500).cut(cyl(27,bottom+line*115,bottom+line*501))
        add('Servo%dBody'%j,body,'Fixed',BLUE,connection='LowerEye / piston')
        add('Servo%dRod'%j,cyl(25,top-line*580,top-line*80),'Fixed',connection='piston / UpperEye',note='500 mm rigid piston rod; translates without resizing')
        add('Servo%dPiston'%j,eye(26.8,25,20,top-line*570,line),'Fixed',GOLD,connection='ServoRod / hydraulic pressure',note='Seal grooves and ports are interface detail')
    for n,key,g,rotation in [('Antirotation','stator_scissors','Fixed',qs),('RotatingDrive','rotor_scissors','Hub',qr)]:
        s=frames[key];rrs(n,s,g,rotation[:,1]);a=s['base'];u=s['base_axis']
        if n=='Antirotation':support=cyl(48,a-[0,0,360],a-[0,0,35])
        else:support=cyl(48,az@np.array([150,0,1900]),a-[*map(float,az[:,0]*35)])
        add(n+'Support',support,g,connection='AirframeCarrier' if g=='Fixed' else 'Mast')
        b=s['end'];rad=s['radial_basis']
        attach=center+rotation@np.array([710,0,-100 if g=='Fixed' else 10])
        add(n+'RingBracket',cyl(30,b-rad*75,attach),g,BLUE,connection=n+'EndFork / swash ring')
    add('HubRing',ring(380,160,440,p['hub_z']-220),'Hub',connection='Mast / blade housings')
    add('HubCap',ring(400,160,40,p['hub_z']+220),'Hub',BLUE,connection='HubRing axial retention')
    blade_shape=foil(p);fold_angles=[0,-72,36,-36,72]
    # Sequential fold: 2,5,3,4. A single progress value is reproducible and reversible.
    fractions=np.zeros(5)
    for k,index in enumerate([1,4,2,3]):fractions[index]=np.clip(fold*4-k,0,1)
    for i in range(5):
        g='Blade%d'%(i+1);angle=psi+i*72;ar=rz(deg(angle))
        s=solve(angle,collective,cc,cs,flap[i],lag[i],p=p,mast_psi=psi)
        rot=s['rotation'];pivot=s['pivot'];loc=lambda pt:pivot+rot@np.array(pt);ls=lambda sh:moved(sh,rot,pivot)
        fixed=lambda pt:ar@np.array(pt)+[0,0,p['hub_z']]
        housing=Part.makeBox(520,440,440,V([300,-220,-220])).cut(Part.makeSphere(190.15,V([600,0,0]))).cut(cyl(100,[600,0,0],[821,0,0]))
        housing=housing.cut(Part.makeBox(200,250,360,V([500,170,-160])))
        add(g+'Housing',moved(housing,ar,[0,0,p['hub_z']]),g,BLUE,connection='HubRing / OuterRace',note='Bolted split housing; opening cap retained radially')
        radial_sphere=lambda r:Part.makeSphere(r,V([0,0,0]),V([1,0,0]))
        outer=radial_sphere(190).cut(radial_sphere(174)).cut(cyl(95,[-250,0,0],[250,0,0]))
        add(g+'OuterRace',moved(outer,ar,pivot),g,connection='Housing / elastomer')
        rubber=[];shims=[]
        for k in range(5):
            r=145+k*5.8
            rubber.append(radial_sphere(r+4.5).cut(radial_sphere(r)).cut(cyl(90,[-250,0,0],[250,0,0])))
            shims.append(radial_sphere(r+5.8).cut(radial_sphere(r+4.5)).cut(cyl(90,[-250,0,0],[250,0,0])))
        add(g+'Elastomer',moved(Part.makeCompound(rubber),ar,pivot),g,RUBBER,connection='InnerRace / Shims / OuterRace',material='Bonded elastomer; supplier compound required',note='Undeformed material envelope; stiffness/strain assessment separate')
        add(g+'Shims',moved(Part.makeCompound(shims),ar,pivot),g,connection='Elastomer')
        add(g+'InnerRace',ls(Part.makeSphere(145)),g,connection='Elastomer / Cuff')
        fp=p['fold_pin']-p['joint']
        cuff=cyl(65,[0,0,0],[fp-350,0,0])
        cuff=cuff.fuse(cyl(85,[335,0,0],[365,0,0]))
        # Horn is integral with the cuff; no overlapping separate structural rod.
        horn=fuse([cyl(40,[280,0,0],[280,p['horn']+100,0]),cyl(40,[280,p['horn']+100,0],[p['horn_x']-p['joint'],p['horn']+100,0])])
        add(g+'CuffHorn',ls(cuff.fuse(horn)),g,connection='InnerRace / PitchUpperFork / FoldClevis')
        # Fixed stop cradle: circular collar leaves pitch free; faces limit flap/lag.
        cradle=[Part.makeBox(185,360,25,V([200,-180,-140])),Part.makeBox(30,360,25,V([335,-180,146]))]
        for sign in [-1,1]:cradle.append(Part.makeBox(30,25,286,V([335,sign*150-12.5,-115])))
        cradle.append(Part.makeBox(30,160,25,V([335,-80,-140])))
        add(g+'StopCradle',moved(fuse(cradle),ar,pivot),g,BLUE,connection=g+'Housing / CuffHorn',note='Lower face -115 mm, upper +146 mm; circular collar provides flap stops while retaining pitch motion')
        for sign in [-1,1]:
            pad=Part.makeBox(30,16,170,V([335,sign*129.5-8,-85]))
            add(g+'LagStop'+str(sign).replace('-','M'),moved(pad,ar,pivot),g,GOLD,connection=g+'StopCradle',note='Replaceable stop face at nominal lag 6 degrees')
        shoe=Part.makeBox(30,80,30,V([335,-40,-115]))
        if not (parked or fold):shoe.translate(V([0,190,-60]))
        add(g+'ParkShoe',moved(shoe,ar,pivot),g,GOLD,connection=g+'StopCradle',note='Captive manual support wedge: fitted after jacking level for folding, stowed during flight')
        line=unit(s['top']-s['bottom'])
        spherical(g+'PitchLower',s['bottom'],qr@rz(deg(i*72))[:,0],line,-qr[:,2],g,'RotorRing')
        spherical(g+'PitchUpper',s['top'],rot[:,0],-line,rot[:,1],g,g+'CuffHorn',reach=100)
        link=add(g+'PitchLink',cyl(20,s['bottom']+line*80,s['top']-line*80),g,connection='PitchLowerEye / PitchUpperEye',note='Fixed centre distance; threaded adjustment envelope')
        for key,value in [('BottomCentre',s['bottom']),('TopCentre',s['top'])]:link.addProperty('App::PropertyVector',key,'Kinematics');setattr(link,key,V(value))
        link.addProperty('App::PropertyAngle','SolvedPitch','Kinematics');link.SolvedPitch=s['theta']
        # Damper brackets bypass the pitch linkage; spherical eyes carry real pins.
        da=fixed([420,-400,0]);db=loc([450,-350,0]);dl=unit(db-da)
        spherical(g+'DamperA',da,ar[:,2],dl,ar[:,1],g,g+'Housing')
        spherical(g+'DamperB',db,rot[:,2],-dl,rot[:,1],g,g+'CuffHorn')
        add(g+'DamperMount',cyl(40,fixed([420,-200,0]),da+ar[:,1]*75),g,connection='Housing / DamperAFork')
        add(g+'DamperLug',cyl(40,loc([450,0,0]),db+rot[:,1]*75),g,connection='CuffHorn / DamperBFork')
        add(g+'DamperBody',cyl(48,da+dl*80,da+dl*350).cut(cyl(22,da+dl*110,da+dl*351)),g,BLUE,connection='DamperAEye / piston')
        add(g+'DamperRod',cyl(20,db-dl*430,db-dl*80),g,connection='piston / DamperBEye',note='350 mm constant rod length')
        add(g+'DamperPiston',eye(21.8,20,20,db-dl*420,dl),g,GOLD,connection='DamperRod / oil chamber')
        plates=[]
        for z in [-130,90]:
            plate=Part.makeBox(550,300,40,V([fp-450,-150,z]))
            for xx in [fp,fp-200]:plate=plate.cut(cyl(55.5,[xx,0,z-1],[xx,0,z+41]))
            plates.append(plate)
        plates.append(Part.makeBox(100,300,260,V([fp-450,-150,-130])))
        add(g+'FoldClevis',ls(fuse(plates)),g,BLUE,connection='CuffHorn / hinge and flight lock')
        foldq=rz(deg(fold_angles[i]*fractions[i]));origin=loc([fp,0,0]);fq=rot@foldq
        tang=Part.makeBox(460,220,160,V([-300,-110,-80]))
        for xx in [0,-200]:tang=tang.cut(cyl(55.5,[xx,0,-81],[xx,0,81]))
        add(g+'FoldTang',moved(tang,fq,origin),g,connection='RootSpar / hinge and lock')
        pin(g+'FoldHinge',origin,rot[:,2],g,r=55,half=145)
        withdraw=320 if parked or fold else 0
        lc=loc([fp-200,0,withdraw]);pin(g+'FlightLock',lc,rot[:,2],g,r=55,half=145)
        # Captive screw sleeve carries the withdrawn pin, not an unconnected pin in space.
        tube=ring(100,83.5,355,130).fuse(ring(100,55.5,15,485))
        add(g+'LockGuide',ls(moved(tube,np.eye(3),[fp-200,0,0])),g,BLUE,connection='FoldClevis / captive flight pin',note='Manual screw withdrawal with trapped-key collar; see operating-state.md')
        # Two webs tie the elevated withdrawal guide back to the top clevis plate.
        for sign in [-1,1]:
            web=Part.makeBox(95,20,140,V([fp-250,sign*105-10,130]))
            add(g+'LockWeb'+str(sign).replace('-','M'),ls(web),g,connection='LockGuide / FoldClevis')
        blade=add(g+'Airfoil',moved(blade_shape,fq,origin),g,BLADE,connection='RootSpar',material='Equivalent composite airfoil envelope',note='Full span; mass model does not use solid density')
        blade.addProperty('App::PropertyMass','EquivalentMass','Mass');blade.EquivalentMass=p['blade_mass']
        blade.addProperty('App::PropertyVector','EquivalentCG','Mass');blade.EquivalentCG=V(origin+fq@np.array([(p['root']+p['radius'])/2-p['fold_pin'],0,0]))
        spar=Part.makeBox(290,180,100,V([-70,-90,-50])).cut(cyl(55.5,[0,0,-51],[0,0,51]))
        add(g+'RootSpar',moved(spar,fq,origin),g,BLADE,connection='FoldTang / Airfoil',material='Equivalent composite insert',note='Insert-to-laminate qualification unresolved')
    for obname in ['RotatingDriveRingBracket']+['Blade%dPitchLowerFork'%i for i in range(1,6)]:
        ob=d.getObject(obname);ob.Shape=ob.Shape.cut(upper_fastener_tool)
    add('AirframeEnvelope',Part.makeBox(13000,3600,3000,V([-6500,-1800,-3000])),'Reference',(.7,.75,.8),material='Boundary only')
    if A.GuiUp:d.AirframeEnvelope.Visibility=False
    d.recompute()
    d.addObject('App::FeaturePython','JointLedger').addProperty('App::PropertyString','JSON','Evidence')
    d.JointLedger.JSON=json.dumps(joints)
    return d,bom

def components(doc):return [o for o in doc.Objects if o.TypeId=='PartDesign::Feature' and o.Name!='AirframeEnvelope']
def save(doc,bom,stem):
    doc.recompute();doc.saveAs(str(OUT/(stem+'.FCStd')))
    Part.export(components(doc),str(OUT/(stem+'.step')))
    with (OUT/'bom.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['CAD_name','assembly','material','connection','note','solids']);w.writerows(bom)
if __name__=='__main__':
    for stem,kwargs in [('rotor-neutral',{}),('rotor-max-input',dict(collective=16,cc=2.12132034356,cs=2.12132034356,psi=36,flap=[6,0,-3,0,3],lag=[-3,0,3,0,0])),('rotor-parked',dict(parked=True)),('rotor-folded',dict(fold=1))]:
        doc,bom=build('C2_'+stem.replace('-','_'),**kwargs);save(doc,bom,stem);print(stem,len(components(doc)),flush=True);A.closeDocument(doc.Name)
