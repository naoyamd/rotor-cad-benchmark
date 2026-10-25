"""Execute in FreeCAD GUI. Saved source models retain all components."""
import FreeCAD as A
import FreeCADGui as G
import Part
from build import OUT,components,V

def style(doc):
    for o in doc.Objects:
        if o.TypeId=='App::DocumentObjectGroup':o.Visibility=True
    for o in components(doc):
        n=o.Name; color=(.67,.72,.79)
        if any(w in n for w in ['Stator','RotorRing','ServoBody','DamperBody']):color=(.15,.35,.65)
        if any(w in n for w in ['Airfoil','RootSpar']):color=(.34,.42,.49)
        if any(w in n for w in ['Ball','Joint','Hinge','Lock','Rolling']):color=(.86,.62,.18)
        if 'Elastomer' in n:color=(.14,.16,.19)
        o.ViewObject.ShapeColor=color;o.ViewObject.LineColor=(.1,.12,.17);o.Visibility=True
    doc.AirframeEnvelope.Visibility=False

def shot(doc,filename,top=False):
    A.setActiveDocument(doc.Name)
    view=G.activeDocument().activeView()
    if top:view.viewTop()
    else:view.viewAxonometric()
    view.fitAll();G.updateGui();view.setAnimationEnabled(False) if hasattr(view,'setAnimationEnabled') else None
    view.saveImage(str(OUT/'images'/filename),1800,1200,'White')

def render():
    docs={}
    for stem in ['rotor-neutral','rotor-max-input','rotor-folded','rotor-parked']:
        path=str(OUT/(stem+'.FCStd'))
        for old in list(A.listDocuments().values()):
            if old.FileName.replace('\\','/')==path.replace('\\','/'):A.closeDocument(old.Name)
        doc=A.openDocument(path);style(doc);doc.recompute();doc.save();docs[stem]=doc
    neutral=docs['rotor-neutral']
    shot(neutral,'assembly.png')
    shot(docs['rotor-folded'],'folded-top.png',True)
    shot(docs['rotor-max-input'],'maximum-control.png')
    for o in components(neutral):
        if 'Airfoil' in o.Name:o.Visibility=False
    shot(neutral,'head-detail.png')
    for o in components(neutral):o.Visibility=True
    # Cutaway documents are illustration-only and are not saved as the assembly.
    selection=[n for n in ['Mast','FixedGuide','SlidingSleeve','SupportSphere','TiltSocket','Stator','RotorRing','LowerRace','UpperRace','BearingRollingElements']]
    for title,names,box in [
        ('swash-section',selection,Part.makeBox(2000,1000,800,V([-1000,0,800]))),
        ('root-section',[o.Name for o in components(neutral) if o.Name.startswith('Blade1') and 'Airfoil' not in o.Name],Part.makeBox(1600,650,800,V([200,0,1800])))]:
        cut=A.newDocument(title.replace('-','_'))
        for name in names:
            source=neutral.getObject(name);sh=source.Shape.common(box)
            if sh.Volume>1e-5:
                o=cut.addObject('PartDesign::Feature',name);o.Shape=sh;o.ViewObject.ShapeColor=source.ViewObject.ShapeColor
        cut.recompute();shot(cut,title+'.png')
        if title=='swash-section':
            G.activeDocument().activeView().viewFront();G.activeDocument().activeView().fitAll();G.updateGui()
            G.activeDocument().activeView().saveImage(str(OUT/'images'/str(title+'.png')),1800,1200,'White')
        A.closeDocument(cut.Name)
    A.setActiveDocument(neutral.Name);G.activeDocument().activeView().viewAxonometric();G.activeDocument().activeView().fitAll();G.updateGui()
    return docs
