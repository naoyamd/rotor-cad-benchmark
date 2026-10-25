import * as THREE from './vendor/three.module.js';
import {OrbitControls} from './vendor/OrbitControls.js';
import {createRig} from './c2-rig.mjs';
import {surfaceFor} from './c2-materials.mjs';
import {pitchPresets, speedPresets} from './presets.mjs';

const $ = id => document.getElementById(id);
const colors = {blue:0x38bee2,gold:0xedc263,silver:0xb0c8df,blade:0x578bc2,rubber:0x253c60};
const stateNames = {neutral:'中立','max-input':'最大入力',parked:'駐機',folded:'折畳み'};
let renderer,scene,camera,orbit,cad,live,rig,selected,ambientLight,fillLight;
let mode='motion',currentState='neutral',view='head',requestId=0;
let playing=false,azimuth=0,displayRpm=speedPresets[0],lastTime=0,lastReadout=0,liveLoading=false;
const cache=new Map();
const vec=a=>new THREE.Vector3(...a);
function meshColor(name){
  if(/Airfoil|RootSpar/.test(name))return colors.blade;
  if(/Elastomer/.test(name))return colors.rubber;
  if(/Ball|Joint|FlightLock|FoldHinge|Pin|Retainer|BearingBalls|Cage|Piston|RaceBolts/.test(name))return colors.gold;
  if(/Stator|RotorRing|ServoBody|DamperBody|Housing|Cradle|Guide|Carrier/.test(name))return colors.blue;
  return colors.silver;
}
async function getData(state){
  if(!cache.has(state))cache.set(state,fetch(`assets/c2/${state}.json`).then(response=>{
    if(!response.ok)throw Error(`HTTP ${response.status}`);return response.json();
  }).catch(error=>{cache.delete(state);throw error;}));
  return cache.get(state);
}
function makeGroup(data){
  const group=new THREE.Group();
  for(const part of data.parts){
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.Float32BufferAttribute(part.vertices,3));
    geometry.setIndex(part.indices);geometry.computeVertexNormals();
    const object=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:meshColor(part.name),metalness:.45,roughness:.48}));
    object.name=part.name;object.userData=part;group.add(object);
  }
  return group;
}
function clearSelection(){if(selected)selected.material.emissive.setHex(0);selected=null;}
function selectPart(object){
  clearSelection();if(!object)return;selected=object;object.material.emissive.setHex(0x00365a);
  $('partName').textContent=object.name;
  $('partDetail').textContent=[object.userData.group,object.userData.connection,object.userData.material,object.userData.note].filter(Boolean).join(' / ');
}
function applyXray(){
  const xray=$('xray').checked;
  ambientLight.color.setHex(xray?0xd7f2f9:0xffffff);ambientLight.groundColor.setHex(xray?0x334f7c:0x575b62);
  fillLight.color.setHex(xray?0x86c1d7:0xffffff);
  const legend=document.querySelectorAll('.explorer-foot>span');
  const labels=xray?['操縦・スワッシュ','関節・ロック','荷重を支える骨格']:['金属：銀・グレー','ブッシュ：ブロンズ','複合材・ゴム：黒系'];
  const swatches=xray?['#38bee2','#edc263','#b0c8df']:['#969da5','#927344','#30343a'];
  for(let i=0;i<3;i++){legend[i].innerHTML=`<i class="legend" style="background:${swatches[i]}"></i>${labels[i]}`;}
  for(const group of [cad,live])group?.traverse(object=>{
    if(!object.isMesh)return;
    const finish=xray?{color:meshColor(object.name),metalness:.45,roughness:.48}:surfaceFor(object.userData);
    object.material.color.setHex(finish.color);object.material.metalness=finish.metalness;object.material.roughness=finish.roughness;
    const solid=/Pin|PitchLink|Rod|FlightLock|FoldHinge|BearingBalls|Piston/.test(object.name);
    object.material.transparent=$('xray').checked&&!solid;
    object.material.opacity=object.material.transparent ? 0.24 : 1;
    object.material.depthWrite=!object.material.transparent;object.material.needsUpdate=true;
  });
}
function disposeGroup(group){group.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});scene.remove(group);}
function applyIsolation(){clearSelection();for(const o of cad?.children??[])o.visible=$('isolate').value==='all'||o.userData.group===$('isolate').value;}
function showError(error){stopMotion();$('loadError').textContent=`表示を更新できませんでした（${error.message}）。モードボタンで再読込できます。原本は下のGitHubリンクから確認できます。`;$('loadError').hidden=false;$('status').textContent='モデルの読み込み・計算エラー';}
async function ensureLive(){
  if(liveLoading)return;
  if(rig){live.visible=mode==='motion';if(mode==='motion')updateLive(true);return;}
  liveLoading=true;if(mode==='motion')$('status').textContent='C2の機構モデルを読み込み中…';
  try{
    const group=makeGroup(await getData('neutral'));
    try{rig=createRig(group);}catch(error){disposeGroup(group);throw error;}
    live=group;live.visible=mode==='motion';scene.add(live);applyXray();
    if(mode==='motion')updateLive(true);
  }catch(error){if(mode==='motion')showError(error);}
  finally{liveLoading=false;}
}
async function loadState(state){
  const ticket=++requestId;currentState=state;clearSelection();
  if(mode==='cad'){$('status').textContent='保存CADを読み込み中…';$('loadError').hidden=true;}
  if(cad)cad.visible=false;
  document.querySelectorAll('[data-state]').forEach(b=>{const active=b.dataset.state===state;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
  try{
    const data=await getData(state);if(ticket!==requestId)return;
    const group=makeGroup(data);if(cad)disposeGroup(cad);cad=group;scene.add(cad);cad.visible=mode==='cad';applyIsolation();applyXray();
    $('partName').textContent='部品を選択';$('partDetail').textContent='3D上の部品をクリック。名前・接続先・設計上の注記を確認できます。';
    if(mode==='cad')$('status').textContent=`${data.parts.length}形状フィーチャ / ${stateNames[state]} / 保存CAD`;
  }catch(error){if(ticket===requestId&&mode==='cad')showError(error);}
}
function updateLive(readout){
  if(!rig)return;
  const collective=+$('collective').value,cyclic=+$('cyclic').value,phase=+$('phase').value*Math.PI/180;
  const {blades,frames}=rig.update({collective,cyclicC:cyclic*Math.cos(phase),cyclicS:cyclic*Math.sin(phase),azimuth});
  if(!readout)return;
  $('collectiveValue').value=`${collective.toFixed(1)}°`;$('cyclicValue').value=`${cyclic.toFixed(1)}°`;$('phaseValue').value=`${$('phase').value}°`;$('azimuthValue').value=`${azimuth.toFixed(1)}°`;
  $('pitches').innerHTML=blades.map((b,i)=>`<div>翼 ${i+1}<strong>${b.theta.toFixed(2)}°</strong></div>`).join('');
  $('pitchSection').setAttribute('transform',`rotate(${-blades[0].theta} 110 38)`);
  $('residual').value=Math.max(...blades.map(b=>b.residual)).toExponential(1);
  $('scissorResidual').value=Math.max(frames.stator_scissors.closure_residual_mm,frames.rotor_scissors.closure_residual_mm).toExponential(1);
  $('status').textContent=`C2閉路解 / 425形状 / 表示速度 ${displayRpm} rpm`;
}
function refresh(){try{updateLive(true);}catch(error){showError(error);}}
function syncPresets(){
  document.querySelectorAll('[data-pitch]').forEach(b=>{const p=pitchPresets[+b.dataset.pitch],active=p.collective===+$('collective').value&&p.cyclic===+$('cyclic').value;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
  document.querySelectorAll('[data-speed]').forEach(b=>{const active=speedPresets[+b.dataset.speed]===displayRpm;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
}
function stopMotion(){playing=false;$('motionPlay').textContent='▶ 回転を開始';$('motionPlay').setAttribute('aria-pressed','false');}
function setMode(next){
  stopMotion();clearSelection();mode=next;const isCad=mode==='cad';
  $('cadControls').hidden=!isCad;$('motionControls').hidden=isCad;
  if(cad)cad.visible=isCad;if(live)live.visible=!isCad;
  for(const [id,active] of [['cadMode',isCad],['motionMode',!isCad]]){$(id).classList.toggle('active',active);$(id).setAttribute('aria-pressed',String(active));}
  $('modeTag').textContent=isCad?'FREECAD → THREE.JS':'C2 / LIVE KINEMATICS';
  $('stageTitle').textContent=isCad?'C2 / SAVED CAD':'C2 / LIVE MOTION';
  $('stageDescription').textContent=isCad?'425形状フィーチャの保存CAD。部品をクリックして調べる。':'C2の閉路計算に合わせて実CAD形状を連続駆動。';
  $('modelNote').textContent=isCad?'保存済みCADの三角形化表示 / 長さ単位 m':'C2閉路解でメッシュ姿勢を更新 / 干渉・強度検証ではありません';
  $('loadError').hidden=true;
  if(isCad){if(cad)$('status').textContent=`425形状フィーチャ / ${stateNames[currentState]} / 保存CAD`;else loadState(currentState);}
  else ensureLive();
}
function setCamera(which){
  view=which;const target=which==='head'?vec([0,0,1.65]):vec([0,0,1.1]);
  camera.position.copy(which==='head'?vec([4.5,-5.5,4.2]):which==='top'?vec([0,-.01,29]):vec([18,-21,17]));
  if(which!=='head')camera.position.sub(target).multiplyScalar(Math.max(1,1.55/camera.aspect)).add(target);
  orbit.target.copy(target);orbit.update();
  document.querySelectorAll('[data-view]').forEach(b=>{const active=b.dataset.view===which;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
}
try{
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor(0x002060,0);renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.5;$('viewport').appendChild(renderer.domElement);
  scene=new THREE.Scene();camera=new THREE.PerspectiveCamera(38,1,.02,150);camera.up.set(0,0,1);orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.minDistance=.35;orbit.maxDistance=65;orbit.maxPolarAngle=Math.PI*.94;
  ambientLight=new THREE.HemisphereLight(0xd7f2f9,0x334f7c,2.1);scene.add(ambientLight);const key=new THREE.DirectionalLight(0xffffff,3);key.position.set(4,-5,9);scene.add(key);fillLight=new THREE.DirectionalLight(0x86c1d7,2);fillLight.position.set(-5,1,4);scene.add(fillLight);
  const grid=new THREE.GridHelper(24,48,0x385c8c,0x1a3d70);grid.rotation.x=Math.PI/2;grid.position.z=-.12;grid.material.transparent=true;grid.material.opacity=.26;scene.add(grid);
  new ResizeObserver(()=>{const w=$('viewport').clientWidth,h=$('viewport').clientHeight;if(!w||!h)return;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();setCamera(view);}).observe($('viewport'));
  const ray=new THREE.Raycaster();let down;
  renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
  renderer.domElement.addEventListener('pointerup',e=>{if(mode!=='cad'||!cad||!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5)return;const r=renderer.domElement.getBoundingClientRect();ray.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),camera);selectPart(ray.intersectObjects(cad.children.filter(o=>o.visible),false)[0]?.object);});
  $('cadMode').onclick=()=>setMode('cad');$('motionMode').onclick=()=>setMode('motion');$('xray').onchange=applyXray;
  document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setCamera(b.dataset.view));
  document.querySelectorAll('[data-state]').forEach(b=>b.onclick=()=>{if(['parked','folded'].includes(b.dataset.state))setCamera('top');loadState(b.dataset.state);});
  $('isolate').onchange=()=>{applyIsolation();setCamera($('isolate').value==='Blade1'?'full':'head');};
  for(const id of ['collective','cyclic','phase','azimuth'])$(id).oninput=()=>{if(id==='azimuth'){stopMotion();azimuth=+$(id).value;}syncPresets();refresh();};
  document.querySelectorAll('[data-pitch]').forEach(b=>b.onclick=()=>{const p=pitchPresets[+b.dataset.pitch];$('collective').value=p.collective;$('cyclic').value=p.cyclic;syncPresets();refresh();});
  document.querySelectorAll('[data-speed]').forEach(b=>b.onclick=()=>{displayRpm=speedPresets[+b.dataset.speed];syncPresets();refresh();});
  $('motionPlay').onclick=()=>{playing=!playing;$('motionPlay').textContent=playing?'Ⅱ 回転を止める':'▶ 回転を開始';$('motionPlay').setAttribute('aria-pressed',String(playing));};
  $('motionReset').onclick=()=>{stopMotion();azimuth=0;displayRpm=speedPresets[0];$('collective').value=12;$('cyclic').value=1.5;$('phase').value=0;$('azimuth').value=0;syncPresets();refresh();setCamera('head');};
  document.addEventListener('visibilitychange',()=>{lastTime=0;if(document.hidden)stopMotion();});
  renderer.domElement.addEventListener('webglcontextlost',e=>{e.preventDefault();showError(Error('3D描画が中断されました。ページを再読み込みしてください'));});
  renderer.setAnimationLoop(time=>{const dt=lastTime?Math.min((time-lastTime)/1000,.1):0;lastTime=time;if(document.hidden)return;if(playing&&mode==='motion'&&rig){azimuth=(azimuth+dt*displayRpm*6)%360;$('azimuth').value=azimuth;try{updateLive(time-lastReadout>120);if(time-lastReadout>120)lastReadout=time;}catch(error){showError(error);}}orbit.update();renderer.render(scene,camera);});
  syncPresets();setCamera('head');setMode('motion');
}catch(error){showError(error);document.querySelectorAll('.explorer button,.explorer input,.explorer select').forEach(e=>e.disabled=true);}
