import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { solve, swash } from './kinematics.mjs';
import {pitchPresets,speedPresets} from './presets.mjs';
import {bi} from './bilingual.mjs';

const $ = id => document.getElementById(id);
const colors = {blue:0x38bee2, gold:0xd6a45a, silver:0xb0c8df, blade:0x578bc2, rubber:0x253c60};
let renderer, scene, camera, orbit, cad = null, live, selected = null;
let mode = 'live', currentState = 'neutral', requestId = 0, running = false, lastTime = 0, lastReadout = 0;
let view = 'head', azimuth = 0, displayRpm = speedPresets[0];
const meshes = [], cache = new Map(), controls = ['collective','cyclic','phase','azimuth'];
const mat = (color, metalness=.45) => new THREE.MeshStandardMaterial({color,metalness,roughness:.48});
const v = (a, scale=1) => new THREE.Vector3(...a).multiplyScalar(scale);
const m4 = q => new THREE.Matrix4().set(q[0],q[1],q[2],0,q[3],q[4],q[5],0,q[6],q[7],q[8],0,0,0,0,1);
const mm = a => v(a,.001);
function mesh(geometry,color,parent){const o = new THREE.Mesh(geometry,mat(color));parent.add(o);return o;}
function cylinder(radius,a,b,color,parent){const o=mesh(new THREE.CylinderGeometry(radius,radius,1,20),color,parent);placeRod(o,a,b);return o;}
function placeRod(o,a,b){const d=b.clone().sub(a);o.position.copy(a).add(b).multiplyScalar(.5);o.scale.y=d.length();o.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),d.normalize());}
function ring(outer,inner,height,color,parent){const shape=new THREE.Shape();shape.absarc(0,0,outer,0,Math.PI*2,false);const hole=new THREE.Path();hole.absarc(0,0,inner,0,Math.PI*2,true);shape.holes.push(hole);const geometry=new THREE.ExtrudeGeometry(shape,{depth:height,bevelEnabled:false,curveSegments:64});geometry.translate(0,0,-height/2);return mesh(geometry,color,parent);}
function partColor(name){if(/Airfoil|RootSpar/.test(name))return colors.blade;if(/Elastomer/.test(name))return colors.rubber;if(/Ball|Joint|FlightLock|FoldHinge|RollingElements|DA$|DB$/.test(name))return colors.gold;if(/Stator|RotorRing|ServoBody|DamperBody/.test(name))return colors.blue;return colors.silver;}

function buildLive(){
  const group = new THREE.Group();scene.add(group);group.visible=false;
  cylinder(.16,v([0,0,0]),v([0,0,2.44]),colors.silver,group);
  ring(.3,.115,.09,colors.silver,group);
  cylinder(.19,v([0,0,.1]),v([0,0,.95]),colors.rubber,group);
  const hub=ring(.38,.16,.21,colors.silver,group);hub.position.z=2.2;
  const plate = new THREE.Group(); group.add(plate);
  const stator=ring(.74,.57,.055,colors.blue,plate);stator.position.z=-.0725;
  const rotor=ring(.74,.57,.055,colors.blue,plate);rotor.position.z=.0225;
  const race=ring(.70,.605,.03,colors.gold,plate);race.position.z=-.03;
  mesh(new THREE.SphereGeometry(.28,24,16),colors.silver,plate);
  const blades = Array.from({length:5},(_,i)=>{
    const root=new THREE.Group();group.add(root);
    cylinder(.065,v([0,0,0]),v([.9,0,0]),colors.silver,root);
    cylinder(.028,v([.24,0,0]),v([.24,.25,0]),colors.silver,root);
    cylinder(.028,v([.24,.25,0]),v([0,.25,0]),colors.silver,root);
    const cuff=mesh(new THREE.BoxGeometry(.45,.3,.22),colors.silver,root);cuff.position.x=.73;
    const pin=cylinder(.043,v([.9,0,-.155]),v([.9,0,.155]),colors.gold,root);
    // shortcut: untwisted blade envelope for explanation; use the CAD tab for the actual NACA0012 geometry.
    const foil=mesh(new THREE.BoxGeometry(7.7,.65,.065),colors.blade,root);foil.position.set(4.85,-.1625,0);
    const tip=mesh(new THREE.BoxGeometry(.16,.65,.07),colors.gold,root);tip.position.set(8.62,-.1625,0);
    const bearing=mesh(new THREE.SphereGeometry(.19,24,16),colors.gold,group);
    const link=cylinder(.019,v([0,0,0]),v([0,0,1]),colors.silver,group);
    link.userData.solid=true;
    const lower=mesh(new THREE.SphereGeometry(.033,12,10),colors.gold,group);
    const upper=mesh(new THREE.SphereGeometry(.033,12,10),colors.gold,group);
    const lug=cylinder(.032,v([0,0,0]),v([0,0,.1]),colors.blue,group);
    return {root,bearing,link,lower,upper,lug};
  });
  const servos=[30,150,270].map(a=>{const r=a*Math.PI/180,bottom=v([.71*Math.cos(r),.71*Math.sin(r),.15]);const body=cylinder(.055,bottom,bottom.clone().add(v([0,0,.55])),colors.blue,group);const rod=cylinder(.025,bottom,bottom.clone().add(v([0,0,1])),colors.silver,group);return {bottom,body,rod};});
  return {group,plate,blades,servos};
}
function liveUpdate(readout=true){
  const collective=+$('collective').value,amp=+$('cyclic').value,phase=+$('phase').value*Math.PI/180;
  const cc=amp*Math.cos(phase),cs=amp*Math.sin(phase),s=swash(collective,cc,cs);
  live.plate.position.copy(mm(s.center));live.plate.quaternion.setFromRotationMatrix(m4(s.q));
  const results=live.blades.map((b,i)=>{
    const state=solve(azimuth+i*72,collective,cc,cs);
    b.root.position.copy(mm(state.pivot));b.root.quaternion.setFromRotationMatrix(m4(state.rotation));
    b.bearing.position.copy(mm(state.pivot));
    b.lower.position.copy(mm(state.bottom));b.upper.position.copy(mm(state.top));
    placeRod(b.link,b.lower.position,b.upper.position);
    const lugBottom=mm(state.bottom).sub(v([0,0,.06]).applyMatrix4(m4(s.q)));
    placeRod(b.lug,lugBottom,b.lower.position);return state;
  });
  live.servos.forEach(({bottom,body,rod})=>{const top=v([bottom.x,bottom.y,-.1]).applyMatrix4(m4(s.q)).add(mm(s.center));const dir=top.clone().sub(bottom).normalize();placeRod(body,bottom,bottom.clone().addScaledVector(dir,.55));placeRod(rod,top.clone().addScaledVector(dir,-.65),top);});
  if(readout){
    $('collectiveValue').value=`${collective.toFixed(1)}°`;$('cyclicValue').value=`${amp.toFixed(1)}°`;$('phaseValue').value=`${$('phase').value}°`;$('azimuthValue').value=`${Math.round(azimuth)}°`;
    $('pitches').innerHTML=results.map((s,i)=>`<div>${bi(`Blade ${i+1}`,`翼 ${i+1}`)}<strong>${s.theta.toFixed(2)}°</strong></div>`).join('');
    $('pitchSection').setAttribute('transform',`rotate(${-results[0].theta} 110 38)`);
    $('residual').value=Math.max(...results.map(s=>s.residual)).toExponential(1);
    $('status').innerHTML=bi('Finite-link solution / θ₀ = 8–16° / cyclic ≤ 3°','有限リンク解 / θ₀ = 8–16° / cyclic ≤ 3°');
  }
}
function applyXray(){
  for(const group of [live?.group,cad])group?.traverse(o=>{
    if(!o.isMesh)return;
    const on=$('xray').checked;
    const solid=o.userData.solid||/PitchLink|Ball|ServoRod/.test(o.name);
    o.material.transparent=on&&!solid;o.material.opacity=on && !solid ? 0.23 : 1;
    o.material.depthWrite=!on||solid;o.material.needsUpdate=true;
    let edge=o.children.find(c=>c.userData.xrayEdge);
    if(on&&!edge){edge=new THREE.LineSegments(new THREE.EdgesGeometry(o.geometry,25),new THREE.LineBasicMaterial({color:0x83ccf1,transparent:true,opacity:.55}));edge.userData.xrayEdge=true;o.add(edge);}
    if(edge)edge.visible=on;
  });
}
function syncPresets(){
  document.querySelectorAll('[data-pitch]').forEach(b=>{const p=pitchPresets[+b.dataset.pitch];const active=p.collective===+$('collective').value&&p.cyclic===+$('cyclic').value;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
  document.querySelectorAll('[data-speed]').forEach(b=>{const active=speedPresets[+b.dataset.speed]===displayRpm;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
}
function clearSelection(){if(selected){selected.material.emissive.setHex(0);selected=null;}}
function selectPart(o){clearSelection();if(!o)return;selected=o;o.material.emissive.setHex(0x00365a);$('partName').innerHTML=bi(o.name,o.name);const metadata=[['Connection','接続',o.userData.connection],['Note','注記',o.userData.note]].filter(([, ,value])=>value);const english=metadata.map(([label,,value])=>`${label}: ${value}`).join(' / ');const japanese=metadata.map(([,label,value])=>`${label}: ${value}`).join(' / ');$('partDetail').innerHTML=metadata.length?bi(english,japanese):bi('See the source BOM for load and connection details.','荷重・接続の詳細は原本BOMを参照してください。');}
function disposeGroup(group){group.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});scene.remove(group);}
async function loadState(state){
  const ticket=++requestId;currentState=state;clearSelection();$('status').innerHTML=bi('Loading CAD geometry…','CAD形状を読み込み中…');$('loadError').hidden=true;
  // Hide the previous state while loading so the selected label never misidentifies geometry.
  if(cad)cad.visible=false;
  document.querySelectorAll('[data-state]').forEach(b=>{b.classList.toggle('active',b.dataset.state===state);b.setAttribute('aria-pressed',String(b.dataset.state===state));});
  try{
    if(!cache.has(state)){const res=await fetch(`assets/${state}.json`);if(!res.ok)throw Error(`HTTP ${res.status}`);cache.set(state,await res.json());}
    if(ticket!==requestId)return;
    const data=cache.get(state),group=new THREE.Group();meshes.length=0;
    for(const part of data.parts){const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(part.vertices,3));geometry.setIndex(part.indices);geometry.computeVertexNormals();const o=mesh(geometry,partColor(part.name),group);o.name=part.name;o.userData=part;meshes.push(o);}
    if(cad)disposeGroup(cad);cad=group;scene.add(cad);cad.visible=mode==='cad';applyIsolation();applyXray();
    $('partName').innerHTML=bi('Select a part','部品を選択');$('partDetail').innerHTML=bi('Click a part in 3D to inspect its name, connections, and design notes.','3D上の部品をクリック。名前・接続先・設計上の注記を確認できます。');
    if(mode==='cad')$('status').innerHTML=bi(`${data.parts.length} parts / ${state} / Directly converted from source`,`${data.parts.length}部品 / ${state} / 原本から直接変換`);
  }catch(error){if(ticket!==requestId)return;$('loadError').innerHTML=bi(`Model could not be loaded (${error.message}). Press a state button to retry. Check the source via the GitHub link below.`,`モデルを読み込めませんでした (${error.message})。状態ボタンを押すと再試行できます。下のGitHubリンクから原本も確認できます。`);$('loadError').hidden=false;$('status').innerHTML=bi('CAD load failed','CAD読み込み失敗');}
}
function applyIsolation(){clearSelection();for(const o of meshes)o.visible=$('isolate').value==='all'||o.userData.group===$('isolate').value;}
function setCamera(which){view=which;const target=which==='head'?v([0,0,1.55]):v([0,0,1.4]);camera.position.copy(which==='head'?v([3.5,-4.7,3.8]):which==='top'?v([0,-.01,24]):v([13,-17,14]));if(which!=='head')camera.position.sub(target).multiplyScalar(Math.max(1,1.55/camera.aspect)).add(target);orbit.target.copy(target);orbit.update();document.querySelectorAll('[data-view]').forEach(b=>{b.classList.toggle('active',b.dataset.view===which);b.setAttribute('aria-pressed',String(b.dataset.view===which));});}
function stop(){running=false;$('play').innerHTML=bi('▶ Start rotation','▶ 回転を開始');$('play').setAttribute('aria-pressed','false');}
function setMode(next){mode=next;const isCad=mode==='cad';stop();$('cadControls').hidden=!isCad;$('liveControls').hidden=isCad;live.group.visible=!isCad;if(cad)cad.visible=isCad;for(const [id,active] of [['cadMode',isCad],['liveMode',!isCad]]){$(id).classList.toggle('active',active);$(id).setAttribute('aria-pressed',String(active));}$('modeTag').innerHTML=bi(isCad?'FREECAD → THREE.JS':'LIVE FINITE-LINK SOLVER',isCad?'FREECAD → THREE.JS':'有限リンク機構のライブ計算');$('stageTitle').innerHTML=bi(isCad?'C1 / ROTOR HEAD':'C1 / HOW IT MOVES',isCad?'C1 / ローターヘッド':'C1 / 動き方');$('stageDescription').innerHTML=bi(isCad?'145-part real CAD geometry. Click a part to inspect.':'Simplified model driven by the original mechanism equations. Blade angles come from the closure.',isCad?'145部品の実CAD形状。部品をクリックして調べる。':'元の機構式で動く簡略モデル。翼角は閉路から計算。');$('modelNote').innerHTML=bi(isCad?'Real CAD triangulation / length unit: m':'Simplified mechanism-explanation geometry / not for interference or aerodynamic evaluation',isCad?'実CADの三角形化表示 / 長さ単位 m':'機構解説用の簡略形状 / 干渉・空力の評価ではありません');$('loadError').hidden=true;if(!isCad)liveUpdate();else if(cad)$('status').innerHTML=bi(`${meshes.length||145} parts / ${currentState} / Directly converted from source`,`${meshes.length||145}部品 / ${currentState} / 原本から直接変換`);else loadState(currentState);}

try{
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor(0x002060,0);renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.5;$('viewport').appendChild(renderer.domElement);
  scene=new THREE.Scene();camera=new THREE.PerspectiveCamera(38,1,.02,150);camera.up.set(0,0,1);orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.minDistance=1;orbit.maxDistance=45;orbit.maxPolarAngle=Math.PI*.94;orbit.enablePan=true;
  scene.add(new THREE.HemisphereLight(0xd7f2f9,0x334f7c,2.1));const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(4,-5,9);scene.add(light);const fill=new THREE.DirectionalLight(0x86c1d7,2);fill.position.set(-5,1,4);scene.add(fill);
  const grid=new THREE.GridHelper(24,48,0x385c8c,0x1a3d70);grid.rotation.x=Math.PI/2;grid.position.z=-.12;grid.material.transparent=true;grid.material.opacity=.26;scene.add(grid);
  const circle=new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(Array.from({length:128},(_,i)=>v([9.3*Math.cos(i*Math.PI/64),9.3*Math.sin(i*Math.PI/64),-.1]))),new THREE.LineBasicMaterial({color:0x38bee2,transparent:true,opacity:.35}));scene.add(circle);
  live=buildLive();setMode('live');applyXray();syncPresets();setCamera('head');
  new ResizeObserver(()=>{const w=$('viewport').clientWidth,h=$('viewport').clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();setCamera(view);}).observe($('viewport'));
  const raycaster=new THREE.Raycaster();let down;
  renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});renderer.domElement.addEventListener('pointerup',e=>{if(mode!=='cad'||!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5)return;const r=renderer.domElement.getBoundingClientRect();raycaster.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),camera);selectPart(raycaster.intersectObjects(meshes.filter(o=>o.visible),false)[0]?.object);});
  $('cadMode').onclick=()=>setMode('cad');$('liveMode').onclick=()=>setMode('live');
  document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setCamera(b.dataset.view));document.querySelectorAll('[data-state]').forEach(b=>b.onclick=()=>{if(b.dataset.state==='folded'||b.dataset.state==='parked'){setCamera('top');$('isolate').value='all';}loadState(b.dataset.state);});
  $('isolate').onchange=()=>{applyIsolation();setCamera($('isolate').value==='Blade1'?'full':'head');};
  controls.forEach(id=>$(id).addEventListener('input',()=>{if(id==='azimuth'){stop();azimuth=+$(id).value;}syncPresets();liveUpdate();}));
  $('play').onclick=()=>{running=!running;$('play').innerHTML=running?bi('Ⅱ Stop rotation','Ⅱ 回転を止める'):bi('▶ Start rotation','▶ 回転を開始');$('play').setAttribute('aria-pressed',String(running));};
  $('xray').onchange=applyXray;
  document.querySelectorAll('[data-pitch]').forEach(b=>b.onclick=()=>{const p=pitchPresets[+b.dataset.pitch];$('collective').value=p.collective;$('cyclic').value=p.cyclic;syncPresets();liveUpdate();});
  document.querySelectorAll('[data-speed]').forEach(b=>b.onclick=()=>{displayRpm=speedPresets[+b.dataset.speed];syncPresets();});
  $('reset').onclick=()=>{stop();$('collective').value=12;$('cyclic').value=1.5;$('phase').value=0;$('azimuth').value=0;azimuth=0;displayRpm=speedPresets[0];syncPresets();liveUpdate();setCamera('head');};
  document.querySelectorAll('[data-demo]').forEach(b=>b.onclick=()=>{if(b.dataset.demo==='fold'){setMode('cad');$('isolate').value='all';loadState('folded');setCamera('top');}else{setMode('live');$('collective').value=b.dataset.demo==='collective'?16:12;$('cyclic').value=b.dataset.demo==='cyclic'?3:0;$('phase').value=0;syncPresets();liveUpdate();setCamera('head');}$('explore').scrollIntoView({behavior:matchMedia('(prefers-reduced-motion: reduce)').matches?'instant':'smooth'});});
  document.addEventListener('visibilitychange',()=>{lastTime=0;});
  renderer.setAnimationLoop(time=>{const dt=lastTime?Math.min((time-lastTime)/1000,.1):0;lastTime=time;if(document.hidden)return;if(running&&mode==='live'){azimuth=(azimuth+dt*displayRpm*6)%360;$('azimuth').value=Math.round(azimuth);liveUpdate(time-lastReadout>120);if(time-lastReadout>120)lastReadout=time;}orbit.update();renderer.render(scene,camera);});
  renderer.domElement.addEventListener('webglcontextlost',e=>{e.preventDefault();stop();$('loadError').innerHTML=bi('3D rendering was interrupted. Reload the page. Design materials are available through the links below.','3D描画が中断されました。ページを再読み込みしてください。設計資料は下のリンクから閲覧できます。');$('loadError').hidden=false;});
  // CAD assets load on demand when the visitor switches tabs.
}catch(error){$('status').innerHTML=bi('3D rendering could not start','3D描画を開始できません');$('loadError').hidden=false;$('loadError').innerHTML=bi(`Open this in a WebGL-capable browser. Design materials are available through the GitHub links below. (${error.message})`,`WebGL対応のブラウザで開いてください。設計資料は下のGitHubリンクからご覧いただけます。 (${error.message})`);document.querySelectorAll('.tabs button,.controls button,.camera-buttons button,.demo').forEach(b=>b.disabled=true);}
