"""Run render() inside FreeCAD GUI; cutaways never overwrite assembly files."""
import FreeCAD as A
import FreeCADGui as G
import Part
from build import OUT,components,build,V

def style(doc,head=False):
    for o in doc.Objects:
        if hasattr(o,'ViewObject'):o.Visibility=True
    for o in components(doc):
        n=o.Name;c=(.65,.72,.80)
        if any(s in n for s in ['Stator','RotorRing','Fork','ArmA','ArmB','Housing','Body','Clevis','Carrier']):c=(.12,.33,.60)
        if any(s in n for s in ['Pin','Ball','Retainer','Hinge','Lock','Shoe']):c=(.89,.65,.23)
        if any(s in n for s in ['Airfoil','Spar']):c=(.28,.36,.43)
        if 'Elastomer' in n:c=(.15,.17,.19)
        o.ViewObject.ShapeColor=c;o.ViewObject.LineColor=(.12,.15,.19)
        o.Visibility=not (head and 'Airfoil' in n)
    if doc.getObject('AirframeEnvelope'):doc.AirframeEnvelope.Visibility=False

def shot(doc,filename,view='iso',camera=None):
    A.setActiveDocument(doc.Name);v=G.activeDocument().activeView()
    if view=='top':v.viewTop()
    elif view=='front':v.viewFront()
    else:v.viewAxonometric()
    v.fitAll()
    if camera:v.setCamera(camera)
    G.updateGui();v.saveImage(str(OUT/'images'/filename),1600,1100,'White')
    return v.getCamera()

def render():
    docs={}
    for stem in ['rotor-neutral','rotor-max-input','rotor-parked','rotor-folded']:
        path=OUT/(stem+'.FCStd')
        for old in list(A.listDocuments().values()):
            if old.FileName.replace('\\','/')==str(path).replace('\\','/'):A.closeDocument(old.Name)
        d=A.openDocument(str(path));style(d);d.recompute();d.save();docs[stem]=d
    d=docs['rotor-neutral'];shot(d,'assembly.png');style(d,True);shot(d,'head-detail.png')
    style(docs['rotor-max-input'],True);shot(docs['rotor-max-input'],'maximum-control.png')
    shot(docs['rotor-folded'],'folded-top.png','top')
    style(docs['rotor-parked'],True);shot(docs['rotor-parked'],'parked-head.png')
    for title,names,box,view in [
        ('swash-section',[o.Name for o in components(d) if any(k in o.Name for k in ['Mast','Guide','SliderSphere','Stator','RotorRing','Race','Bearing']) and 'Index' not in o.Name],Part.makeBox(2000,1000,850,V([-1000,0,700])),'iso'),
        ('root-section',[o.Name for o in components(d) if o.Name.startswith('Blade1') and 'Airfoil' not in o.Name],Part.makeBox(1900,650,1100,V([200,0,1700])),'iso'),
        ('joint-detail',[o.Name for o in components(d) if o.Name.startswith('Blade1PitchUpper') or o.Name=='Blade1PitchLink'],Part.makeBox(400,400,400,V([400,44,2052])),'iso')]:
        cut=A.newDocument('C2_'+title.replace('-','_'))
        for name in names:
            source=d.getObject(name);s=source.Shape if box is None else source.Shape.common(box)
            if s.Volume>1e-5:
                o=cut.addObject('PartDesign::Feature',name);o.Shape=s;o.ViewObject.ShapeColor=source.ViewObject.ShapeColor
                o.ViewObject.Deviation=.01;o.ViewObject.AngularDeflection=10
        cut.recompute();shot(cut,title+'.png',view);A.closeDocument(cut.Name)
    A.setActiveDocument(d.Name);style(d,True);G.activeDocument().activeView().viewAxonometric();G.activeDocument().activeView().fitAll()
    return docs

def animation():
    camera=None
    for k in range(12):
        d,_=build('MotionFrame',cc=3,psi=k*30)
        style(d,True);camera=shot(d,'motion-%02d.png'%k,camera=camera)
        A.closeDocument(d.Name)
    print('12 BREP motion frames complete')
