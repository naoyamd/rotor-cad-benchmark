import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as THREE from '../vendor/three.module.js';
import {P} from '../kinematics.mjs';
import {createRig, EXPECTED_NAMES, RING_BRACKET_APPROXIMATION_SAMPLE_MAX_MM} from '../c2-rig.mjs';

const asset = JSON.parse(await readFile(new URL('../assets/c2/neutral.json', import.meta.url), 'utf8'));

function makeGroup() {
  const group = new THREE.Group();
  for (const part of asset.parts) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(part.vertices, 3));
    geometry.setIndex(part.indices);
    const mesh = new THREE.Mesh(geometry);
    mesh.name = part.name;
    group.add(mesh);
  }
  return group;
}

function meshes(group) {
  const result = new Map();
  group.traverse(object => { if (object.isMesh) result.set(object.name, object); });
  return result;
}

function assertIdentity(matrix, tolerance = 1e-8) {
  const expected = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  for (let i = 0; i < expected.length; i += 1) {
    assert.ok(Math.abs(matrix.elements[i] - expected[i]) <= tolerance, `matrix element ${i}`);
  }
}

function distance(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function transformPoint(matrix, pointMm) {
  const e = matrix.elements;
  const [x, y, z] = pointMm.map(value => value / 1000);
  return [
    (e[0] * x + e[4] * y + e[8] * z + e[12]) * 1000,
    (e[1] * x + e[5] * y + e[9] * z + e[13]) * 1000,
    (e[2] * x + e[6] * y + e[10] * z + e[14]) * 1000,
  ];
}

function multiply3(a, v) {
  return [
    a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
    a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
    a[6] * v[0] + a[7] * v[1] + a[8] * v[2],
  ];
}

function unit(vector) {
  const length = Math.hypot(...vector);
  return vector.map(value => value / length);
}

function subtract(a, b) {
  return a.map((value, index) => value - b[index]);
}

function servoRodEndpoints(frames) {
  const angle = 30 * Math.PI / 180;
  const er = [Math.cos(angle), Math.sin(angle), 0];
  const bottom = [er[0] * 710, er[1] * 710, 150];
  const top = frames.center.map((value, index) => value + multiply3(frames.stator_rotation, [er[0] * 710, er[1] * 710, -190])[index]);
  const line = unit(subtract(top, bottom));
  return [subtract(top, line.map(value => value * 580)), subtract(top, line.map(value => value * 80))];
}

function damperRodEndpoints(blade, azimuth) {
  const angle = azimuth * Math.PI / 180;
  const ar = [Math.cos(angle), -Math.sin(angle), 0, Math.sin(angle), Math.cos(angle), 0, 0, 0, 1];
  const daLocal = multiply3(ar, [420, -400, 0]);
  const da = [daLocal[0], daLocal[1], 2200];
  const db = blade.pivot.map((value, index) => value + multiply3(blade.rotation, [450, -350, 0])[index]);
  const line = unit(subtract(db, da));
  return [subtract(db, line.map(value => value * 430)), subtract(db, line.map(value => value * 80))];
}

test('C2 rig covers every neutral mesh and rejects unknown names', () => {
  assert.equal(asset.parts.length, 425);
  assert.equal(EXPECTED_NAMES.size, 425);
  assert.deepEqual(new Set(asset.parts.map(part => part.name)), EXPECTED_NAMES);
  const group = makeGroup();
  assert.doesNotThrow(() => createRig(group));
  const unknown = new THREE.Mesh(new THREE.BufferGeometry());
  unknown.name = 'UnknownC2Part';
  group.add(unknown);
  assert.throws(() => createRig(group), /Unknown C2 mesh name/);
});

test('neutral update is identity and never mutates source mesh geometry', () => {
  const group = makeGroup();
  const before = new Map();
  for (const [name, mesh] of meshes(group)) {
    before.set(name, Array.from(mesh.geometry.attributes.position.array));
  }
  const rig = createRig(group);
  rig.update({collective: 12, cyclicC: 0, cyclicS: 0, azimuth: 0});
  for (const [name, mesh] of meshes(group)) {
    assertIdentity(mesh.matrix);
    assert.deepEqual(Array.from(mesh.geometry.attributes.position.array), before.get(name), name);
  }
});

test('fixed anchors stay fixed while swash, housing, and cuff move', () => {
  const group = makeGroup();
  const rig = createRig(group);
  const parts = meshes(group);
  rig.update({collective: 12, cyclicC: 0, cyclicS: 0, azimuth: 0});
  const fixed = ['AirframeCarrier', 'FixedGuide', 'MastIndexBracket', 'MastIndexPin'];
  for (const name of fixed) assertIdentity(parts.get(name).matrix);
  rig.update({collective: 15, cyclicC: 1, cyclicS: -1, azimuth: 35});
  for (const name of fixed) assertIdentity(parts.get(name).matrix);
  assert.ok(!parts.get('MastFlange').matrix.equals(new THREE.Matrix4()));
  assert.ok(!parts.get('HubRing').matrix.equals(new THREE.Matrix4()));
  assert.ok(!parts.get('StatorSocket').matrix.equals(new THREE.Matrix4()));
  assert.ok(!parts.get('RotorRing').matrix.equals(new THREE.Matrix4()));
  assert.ok(!parts.get('Blade1Housing').matrix.equals(new THREE.Matrix4()));
  assert.ok(!parts.get('Blade1CuffHorn').matrix.equals(new THREE.Matrix4()));
});

test('finite-link endpoints and rigid rods remain valid over flight samples', () => {
  const group = makeGroup();
  const rig = createRig(group);
  const neutral = rig.update({collective: 12, cyclicC: 0, cyclicS: 0, azimuth: 0});
  const neutralServoRod = servoRodEndpoints(neutral.frames);
  const neutralDamperRod = damperRodEndpoints(neutral.blades[0], 0);
  const samples = [
    {collective: 8, cyclicC: 0, cyclicS: 0, azimuth: 0},
    {collective: 12, cyclicC: 1.5, cyclicS: 0, azimuth: 72},
    {collective: 16, cyclicC: -3 / Math.sqrt(2), cyclicS: 3 / Math.sqrt(2), azimuth: 144},
  ];
  for (const controls of samples) {
    const result = rig.update(controls);
    const parts = meshes(group);
    for (const blade of result.blades) {
      assert.ok(Math.abs(distance(blade.top, blade.bottom) - P.link_length) < 1e-6);
      assert.ok(blade.residual < 1e-6);
    }
    const currentPitch = result.blades[0];
    assertPointClose(transformPoint(parts.get('Blade1PitchLink').matrix, neutral.blades[0].bottom), currentPitch.bottom);
    assertPointClose(transformPoint(parts.get('Blade1PitchLink').matrix, neutral.blades[0].top), currentPitch.top);
    const currentServoRod = servoRodEndpoints(result.frames);
    assertPointClose(transformPoint(parts.get('Servo0Rod').matrix, neutralServoRod[0]), currentServoRod[0]);
    assertPointClose(transformPoint(parts.get('Servo0Rod').matrix, neutralServoRod[1]), currentServoRod[1]);
    const currentDamperRod = damperRodEndpoints(currentPitch, controls.azimuth);
    assertPointClose(transformPoint(parts.get('Blade1DamperRod').matrix, neutralDamperRod[0]), currentDamperRod[0]);
    assertPointClose(transformPoint(parts.get('Blade1DamperRod').matrix, neutralDamperRod[1]), currentDamperRod[1]);
    assert.ok(Math.abs(distance(currentServoRod[0], currentServoRod[1]) - 500) < 1e-6);
    assert.ok(Math.abs(distance(currentDamperRod[0], currentDamperRod[1]) - 350) < 1e-6);
    for (const scissors of [result.frames.stator_scissors, result.frames.rotor_scissors]) {
      assert.ok(Math.abs(distance(scissors.base, scissors.elbow) - 550) < 1e-6);
      assert.ok(Math.abs(distance(scissors.elbow, scissors.end) - 510) < 1e-6);
    }
    for (const name of ['Servo0Rod', 'Blade1PitchLink', 'Blade1DamperRod']) {
      assert.ok(Math.abs(parts.get(name).matrix.determinant() - 1) < 1e-8, name);
    }
  }
});

test('ring bracket fixed-length approximation reports its sampled endpoint limit', () => {
  const group = makeGroup();
  const rig = createRig(group);
  let max = 0;
  for (const collective of [8, 12, 16]) {
    for (const cyclic of [0, 1.5, 3]) {
      for (let phase = 0; phase < 360; phase += 45) {
        const phaseRad = phase * Math.PI / 180;
        for (let azimuth = 0; azimuth < 360; azimuth += 30) {
          const result = rig.update({
            collective,
            cyclicC: cyclic * Math.cos(phaseRad),
            cyclicS: cyclic * Math.sin(phaseRad),
            azimuth,
          });
          max = Math.max(max, result.ringBracketApproximation.max_mm);
          assert.ok(result.ringBracketApproximation.max_mm <= RING_BRACKET_APPROXIMATION_SAMPLE_MAX_MM);
        }
      }
    }
  }
  assert.ok(max > 1, `sampled maximum unexpectedly small: ${max}`);
});

test('fold pins follow the actual 900 mm root offset while stop cradles follow housing yaw',()=>{
  const group=makeGroup(),rig=createRig(group),parts=meshes(group);
  const neutral=rig.update({collective:12,cyclicC:0,cyclicS:0,azimuth:0});
  const current=rig.update({collective:16,cyclicC:2,cyclicS:2,azimuth:17.3});
  const local=(blade,x)=>multiply3(blade.rotation,[x,0,0]).map((v,i)=>v+blade.pivot[i]);
  for(let i=0;i<5;i++){
    for(const [name,x] of [['FoldHinge',P.fold_pin-P.joint],['FlightLock',P.fold_pin-P.joint-200]]){
      assertPointClose(transformPoint(parts.get(`Blade${i+1}${name}`).matrix,local(neutral.blades[i],x)),local(current.blades[i],x));
    }
    const housing=parts.get(`Blade${i+1}Housing`).matrix.elements;
    const cradle=parts.get(`Blade${i+1}StopCradle`).matrix.elements;
    for(let j=0;j<16;j++)assert.ok(Math.abs(housing[j]-cradle[j])<1e-8);
  }
});

function assertPointClose(actual, expected, tolerance = 1e-6) {
  assert.ok(distance(actual, expected) <= tolerance, `${actual} != ${expected}`);
}
