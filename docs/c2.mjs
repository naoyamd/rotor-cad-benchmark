import * as THREE from './vendor/three.module.js';
import {OrbitControls} from './vendor/OrbitControls.js';

const $ = id => document.getElementById(id);
const colors = {blue: 0x38bee2, gold: 0xedc263, silver: 0xb0c8df, blade: 0x578bc2, rubber: 0x253c60};
const stateNames = {neutral: '中立', 'max-input': '最大入力', parked: '駐機', folded: '折畳み'};
const motionSpeeds = [1500, 700, 300];
let renderer, scene, camera, orbit, cad = null, selected = null;
let mode = 'motion', currentState = 'neutral', view = 'head', requestId = 0;
let motionFrame = 0, motionPlaying = false, motionSpeed = 0, motionTimer = 0;
const meshes = [], cache = new Map();

const material = color => new THREE.MeshStandardMaterial({color, metalness: .45, roughness: .48});
const vec = (a, scale = 1) => new THREE.Vector3(...a).multiplyScalar(scale);
const meshColor = part => {
  const name = part.name || '';
  if (/Airfoil|RootSpar/.test(name)) return colors.blade;
  if (/Elastomer/.test(name)) return colors.rubber;
  if (/Ball|Joint|FlightLock|FoldHinge|Pin|Retainer|BearingBalls|Cage|Piston|RaceBolts/.test(name)) return colors.gold;
  if (/Stator|RotorRing|ServoBody|DamperBody|Housing|Cradle|Guide|Carrier/.test(name)) return colors.blue;
  return colors.silver;
};

function makeMesh(geometry, color, parent) {
  const object = new THREE.Mesh(geometry, material(color));
  parent.add(object);
  return object;
}

function clearSelection() {
  if (selected) selected.material.emissive.setHex(0);
  selected = null;
}

function selectPart(object) {
  clearSelection();
  if (!object) {
    $('partName').textContent = '部品を選択';
    $('partDetail').textContent = '3D上の部品をクリック。名前・接続先・設計上の注記を確認できます。';
    return;
  }
  selected = object;
  selected.material.emissive.setHex(0x00365a);
  const part = object.userData;
  $('partName').textContent = part.name;
  $('partDetail').textContent = [part.group && `構成: ${part.group}`, part.connection, part.material, part.note]
    .filter(Boolean).join(' / ') || '原本BOMに追加の注記はありません。';
}

function applyIsolation() {
  const group = $('isolate').value;
  for (const object of meshes) object.visible = group === 'all' || object.userData.group === group;
}

function applyXray() {
  const enabled = $('xray').checked;
  for (const object of meshes) {
    const solid = /Pin|PitchLink|Rod|FlightLock|FoldHinge|BearingBalls|Piston/.test(object.name);
    object.material.transparent = enabled && !solid;
    object.material.opacity = enabled && !solid ? .24 : 1;
    object.material.depthWrite = !enabled || solid;
    object.material.needsUpdate = true;
  }
}

function disposeGroup(group) {
  group.traverse(object => {
    if (object.geometry) object.geometry.dispose();
    if (object.material) object.material.dispose();
  });
  scene.remove(group);
}

async function loadState(state) {
  const ticket = ++requestId;
  currentState = state;
  clearSelection();
  $('status').textContent = 'CAD形状を読み込み中…';
  $('loadError').hidden = true;
  if (cad) cad.visible = false;
  document.querySelectorAll('[data-state]').forEach(button => {
    const active = button.dataset.state === state;
    button.classList.toggle('active', active);
    button.setAttribute('aria-pressed', String(active));
  });
  try {
    if (!cache.has(state)) {
      const response = await fetch(`assets/c2/${state}.json`);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      cache.set(state, await response.json());
    }
    if (ticket !== requestId) return;
    const data = cache.get(state);
    const group = new THREE.Group();
    meshes.length = 0;
    for (const part of data.parts) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(part.vertices, 3));
      geometry.setIndex(part.indices);
      geometry.computeVertexNormals();
      const object = makeMesh(geometry, meshColor(part), group);
      object.name = part.name;
      object.userData = part;
      meshes.push(object);
    }
    if (cad) disposeGroup(cad);
    cad = group;
    scene.add(cad);
    cad.visible = mode === 'cad';
    applyIsolation();
    applyXray();
    $('partName').textContent = '部品を選択';
    $('partDetail').textContent = '3D上の部品をクリック。名前・接続先・設計上の注記を確認できます。';
    if (mode === 'cad') $('status').textContent = `${data.parts.length}形状フィーチャ / ${stateNames[state]} / 原本から直接変換`;
  } catch (error) {
    if (ticket !== requestId) return;
    $('loadError').textContent = `モデルを読み込めませんでした（${error.message}）。状態ボタンで再試行できます。GitHubには原本があります。`;
    $('loadError').hidden = false;
    $('status').textContent = 'CAD読み込み失敗';
  }
}

function setCamera(which) {
  view = which;
  const target = which === 'head' ? vec([0, 0, 1.65]) : vec([0, 0, 1.1]);
  const position = which === 'head' ? vec([4.5, -5.5, 4.2]) : which === 'top' ? vec([0, -.01, 29]) : vec([18, -21, 17]);
  camera.position.copy(position);
  if (which !== 'head') camera.position.sub(target).multiplyScalar(Math.max(1, 1.55 / camera.aspect)).add(target);
  orbit.target.copy(target);
  orbit.update();
  document.querySelectorAll('[data-view]').forEach(button => {
    const active = button.dataset.view === which;
    button.classList.toggle('active', active);
    button.setAttribute('aria-pressed', String(active));
  });
}

function setMode(next) {
  mode = next;
  const isCad = mode === 'cad';
  $('cadControls').hidden = !isCad;
  $('motionControls').hidden = isCad;
  $('viewport').hidden = !isCad;
  $('motionViewport').hidden = isCad;
  $('xrayOption').hidden = !isCad;
  $('cameraButtons').hidden = !isCad;
  if (cad) cad.visible = isCad;
  $('cadMode').classList.toggle('active', isCad);
  $('cadMode').setAttribute('aria-pressed', String(isCad));
  $('motionMode').classList.toggle('active', !isCad);
  $('motionMode').setAttribute('aria-pressed', String(!isCad));
  $('modeTag').textContent = isCad ? 'FREECAD → THREE.JS' : '12 CAD-GENERATED FRAMES';
  $('stageTitle').textContent = isCad ? 'C2 / ROTOR HEAD' : 'C2 / FINITE CAD MOTION';
  $('stageDescription').textContent = isCad
    ? '425形状フィーチャの実CAD。部品をクリックして調べる。'
    : '12枚の実CAD生成画像を手動スクラブ・再生。連続解ではありません。';
  $('modelNote').textContent = isCad ? '実CADの三角形化表示 / 長さ単位 m' : '12サンプルの実CAD PNG / 連続ソルバーではありません';
  $('loadError').hidden = true;
  if (isCad && cad) $('status').textContent = `425形状フィーチャ / ${stateNames[currentState]} / 原本から直接変換`;
  if (!isCad) setMotionFrame(motionFrame);
}

function setMotionFrame(frame) {
  motionFrame = Math.max(0, Math.min(11, Number(frame)));
  const label = String(motionFrame + 1).padStart(2, '0');
  $('motionImage').src = `assets/c2/motion-${String(motionFrame).padStart(2, '0')}.png`;
  $('motionImage').alt = `C2実CADから生成した動作フレーム ${motionFrame + 1} / 12`;
  $('frameRange').value = String(motionFrame);
  $('frameValue').value = `${label} / 12`;
  $('frameChip').textContent = `FRAME ${label} / 12`;
  $('status').textContent = `CAD生成フレーム ${label} / 12`;
}

function stopMotion() {
  motionPlaying = false;
  window.clearTimeout(motionTimer);
  $('motionPlay').textContent = '▶ 再生';
  $('motionPlay').setAttribute('aria-pressed', 'false');
}

function scheduleMotion() {
  window.clearTimeout(motionTimer);
  if (!motionPlaying) return;
  motionTimer = window.setTimeout(() => {
    setMotionFrame((motionFrame + 1) % 12);
    scheduleMotion();
  }, motionSpeeds[motionSpeed]);
}

function toggleMotion() {
  motionPlaying = !motionPlaying;
  $('motionPlay').textContent = motionPlaying ? 'Ⅱ 停止' : '▶ 再生';
  $('motionPlay').setAttribute('aria-pressed', String(motionPlaying));
  scheduleMotion();
}

function init() {
  setMotionFrame(0);
  $('frameRange').oninput = event => { stopMotion(); setMotionFrame(event.target.value); };
  $('motionPlay').onclick = toggleMotion;
  $('motionReset').onclick = () => { stopMotion(); setMotionFrame(0); };
  document.querySelectorAll('[data-motion-speed]').forEach(button => button.onclick = () => {
    motionSpeed = Number(button.dataset.motionSpeed);
    document.querySelectorAll('[data-motion-speed]').forEach(other => {
      const active = other === button;
      other.classList.toggle('active', active);
      other.setAttribute('aria-pressed', String(active));
    });
    scheduleMotion();
  });
  try {
    renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x002060, 0);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.5;
    $('viewport').appendChild(renderer.domElement);
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(38, 1, .02, 150);
    camera.up.set(0, 0, 1);
    orbit = new OrbitControls(camera, renderer.domElement);
    orbit.enableDamping = true;
    orbit.minDistance = .35;
    orbit.maxDistance = 65;
    orbit.maxPolarAngle = Math.PI * .94;
    orbit.enablePan = true;
    scene.add(new THREE.HemisphereLight(0xd7f2f9, 0x334f7c, 2.1));
    const key = new THREE.DirectionalLight(0xffffff, 3);
    key.position.set(4, -5, 9);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0x86c1d7, 2);
    fill.position.set(-5, 1, 4);
    scene.add(fill);
    const grid = new THREE.GridHelper(24, 48, 0x385c8c, 0x1a3d70);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -.12;
    grid.material.transparent = true;
    grid.material.opacity = .26;
    scene.add(grid);
    const circlePoints = Array.from({length: 128}, (_, i) => vec([9.3 * Math.cos(i * Math.PI / 64), 9.3 * Math.sin(i * Math.PI / 64), -.1]));
    scene.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(circlePoints), new THREE.LineBasicMaterial({color: colors.blue, transparent: true, opacity: .35})));
    setMode('motion');
    setCamera('head');
    new ResizeObserver(() => {
      const width = $('viewport').clientWidth || 1;
      const height = $('viewport').clientHeight || 1;
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      setCamera(view);
    }).observe($('viewport'));
    const raycaster = new THREE.Raycaster();
    let down = null;
    renderer.domElement.addEventListener('pointerdown', event => { down = [event.clientX, event.clientY]; });
    renderer.domElement.addEventListener('pointerup', event => {
      if (mode !== 'cad' || !down || Math.hypot(event.clientX - down[0], event.clientY - down[1]) > 5) return;
      const rect = renderer.domElement.getBoundingClientRect();
      raycaster.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1), camera);
      selectPart(raycaster.intersectObjects(meshes.filter(object => object.visible), false)[0]?.object);
    });
    $('cadMode').onclick = () => {
      stopMotion();
      setMode('cad');
      if (!cad) loadState(currentState);
    };
    $('motionMode').onclick = () => setMode('motion');
    document.querySelectorAll('[data-view]').forEach(button => button.onclick = () => setCamera(button.dataset.view));
    document.querySelectorAll('[data-state]').forEach(button => button.onclick = () => {
      if (button.dataset.state === 'folded' || button.dataset.state === 'parked') setCamera('top');
      loadState(button.dataset.state);
    });
    $('isolate').onchange = () => { applyIsolation(); setCamera($('isolate').value === 'Blade1' ? 'full' : 'head'); };
    $('xray').onchange = applyXray;
    document.addEventListener('visibilitychange', () => { if (document.hidden) stopMotion(); });
    renderer.setAnimationLoop(() => {
      orbit.update();
      renderer.render(scene, camera);
    });
    window.addEventListener('beforeunload', stopMotion);
  } catch (error) {
    $('status').textContent = 'PNG動作ビューア / WebGLなし';
    $('loadError').hidden = mode === 'motion';
    $('loadError').textContent = `WebGL対応のブラウザで開いてください。下のGitHubリンクから原本を確認できます。（${error.message}）`;
    $('cadMode').disabled = true;
    // The PNG motion viewer stays usable when WebGL is unavailable.
  }
}

init();
