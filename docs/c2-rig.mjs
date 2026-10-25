import * as THREE from './vendor/three.module.js';
import {P, deg, rz} from './kinematics.mjs';
import {solve, swashFrames} from './c2-kinematics.mjs';

const IDENTITY = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const HUB_Z = P.hub_z;
const FOLD_PIN = P.fold_pin - P.joint;
// Observed maximum endpoint error (1.2763 mm, rounded up here) on c=8/12/16,
// |cyclic|=0/1.5/3, 45-degree cyclic phase, and 30-degree mast-azimuth
// samples. The current value is returned by update(); this is a display
// approximation, not a CAD tolerance or a collision guarantee.
export const RING_BRACKET_APPROXIMATION_SAMPLE_MAX_MM = 1.3;

const SERVO_PARTS = [
  'LowerBall', 'LowerEye', 'LowerSpacerL', 'LowerSpacerR', 'LowerFork',
  'LowerPin', 'LowerPinHead', 'LowerPinRetainer',
  'UpperBall', 'UpperEye', 'UpperSpacerL', 'UpperSpacerR', 'UpperFork',
  'UpperPin', 'UpperPinHead', 'UpperPinRetainer', 'Body', 'Rod', 'Piston',
];

const ANTIROTATION_PARTS = [
  'ArmA', 'ArmB', 'BasePin', 'BasePinHead', 'BasePinRetainer', 'ElbowPin',
  'ElbowPinHead', 'ElbowPinRetainer', 'BaseBearing', 'EndBall', 'EndEye',
  'EndSpacerL', 'EndSpacerR', 'EndFork', 'EndPin', 'EndPinHead',
  'EndPinRetainer', 'Support', 'RingBracket',
];

const ROTATING_DRIVE_PARTS = [
  'ArmA', 'ArmB', 'BasePin', 'BasePinHead', 'BasePinRetainer', 'ElbowPin',
  'ElbowPinHead', 'ElbowPinRetainer', 'BaseBearing', 'EndBall', 'EndEye',
  'EndSpacerL', 'EndSpacerR', 'EndFork', 'EndPin', 'EndPinHead',
  'EndPinRetainer', 'Support', 'RingBracket',
];

const BLADE_PARTS = [
  'Housing', 'OuterRace', 'Elastomer', 'Shims', 'InnerRace', 'CuffHorn',
  'StopCradle', 'LagStopM1', 'LagStop1', 'ParkShoe',
  'PitchLowerBall', 'PitchLowerEye', 'PitchLowerSpacerL', 'PitchLowerSpacerR',
  'PitchLowerFork', 'PitchLowerPin', 'PitchLowerPinHead',
  'PitchLowerPinRetainer', 'PitchUpperBall', 'PitchUpperEye',
  'PitchUpperSpacerL', 'PitchUpperSpacerR', 'PitchUpperFork', 'PitchUpperPin',
  'PitchUpperPinHead', 'PitchUpperPinRetainer', 'PitchLink',
  'DamperABall', 'DamperAEye', 'DamperASpacerL', 'DamperASpacerR',
  'DamperAFork', 'DamperAPin', 'DamperAPinHead', 'DamperAPinRetainer',
  'DamperBBall', 'DamperBEye', 'DamperBSpacerL', 'DamperBSpacerR',
  'DamperBFork', 'DamperBPin', 'DamperBPinHead', 'DamperBPinRetainer',
  'DamperMount', 'DamperLug', 'DamperBody', 'DamperRod', 'DamperPiston',
  'FoldClevis', 'FoldTang', 'FoldHinge', 'FoldHingeHead',
  'FoldHingeRetainer', 'FlightLock', 'FlightLockHead',
  'FlightLockRetainer', 'LockGuide', 'LockWebM1', 'LockWeb1', 'Airfoil',
  'RootSpar',
];

const FIXED_NAMES = [
  'MastIndexBracket', 'MastIndexPin', 'MastIndexGuide', 'AirframeCarrier',
  'FixedGuide', 'GuideBridge', 'CarrierSpoke0', 'CarrierSpoke1',
  'CarrierSpoke2',
  ...[0, 1, 2].flatMap(index => SERVO_PARTS.map(part => `Servo${index}${part}`)),
  ...ANTIROTATION_PARTS.map(part => `Antirotation${part}`),
];

const SWASH_NAMES = [
  'GuideBushM145', 'GuideBush100', 'SliderSphere', 'StatorSocket',
  'SocketSplitBolts', 'LowerRace', 'UpperRace', 'BearingBalls',
  'BearingCage', 'RotorRing', 'LowerRaceBolts', 'UpperRaceBolts',
];

const HUB_NAMES = [
  'Mast', 'MastFlange',
  ...ROTATING_DRIVE_PARTS.map(part => `RotatingDrive${part}`),
  'HubRing', 'HubCap',
];

const EXPECTED_NAMES = new Set([
  ...FIXED_NAMES,
  ...SWASH_NAMES,
  ...HUB_NAMES,
  ...[1, 2, 3, 4, 5].flatMap(index => BLADE_PARTS.map(part => `Blade${index}${part}`)),
]);

if (EXPECTED_NAMES.size !== 425) {
  throw new Error(`C2 rig map must contain 425 names; got ${EXPECTED_NAMES.size}`);
}

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = a => Math.hypot(a[0], a[1], a[2]);

function unit(value, label = 'vector') {
  const length = norm(value);
  if (!(length > 1e-12) || !Number.isFinite(length)) throw new Error(`${label} must be non-zero`);
  return scale(value, 1 / length);
}

function vector(value, label) {
  if (!value || value.length !== 3) throw new Error(`${label} must be a 3-vector`);
  const result = [Number(value[0]), Number(value[1]), Number(value[2])];
  if (!result.every(Number.isFinite)) throw new Error(`${label} must be finite`);
  return result;
}

function matrix(value, label) {
  if (value && value.length === 9) {
    const result = Array.from(value, Number);
    if (result.every(Number.isFinite)) return result;
  }
  // Accepting Matrix3 here keeps the rig usable by a caller that wraps the
  // solver output in Three.js; solver arrays themselves are row-major.
  if (value?.elements?.length === 9) {
    const e = Array.from(value.elements, Number);
    return [e[0], e[3], e[6], e[1], e[4], e[7], e[2], e[5], e[8]];
  }
  throw new Error(`${label} must be a finite 3x3 matrix`);
}

function multiply3(a, b) {
  return [
    a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
    a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
    a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
    a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
    a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
    a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
    a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
    a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
    a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
  ];
}

function multiplyVec3(a, v) {
  return [
    a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
    a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
    a[6] * v[0] + a[7] * v[1] + a[8] * v[2],
  ];
}

function column(a, index) {
  return [a[index], a[index + 3], a[index + 6]];
}

function columns(x, y, z) {
  return [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
}

function transpose(a) {
  return [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]];
}

function frame(rotation = IDENTITY, translation = [0, 0, 0]) {
  return {r: matrix(rotation, 'rotation'), t: vector(translation, 'translation')};
}

function identityFrame() {
  return frame(IDENTITY, [0, 0, 0]);
}

function axisFrame(center, axis, radial) {
  const z = unit(axis, 'axis');
  let x = radial ? vector(radial, 'radial') : (Math.abs(z[0]) < 0.85 ? [1, 0, 0] : [0, 1, 0]);
  x = unit(sub(x, scale(z, dot(z, x))), 'radial basis');
  return frame(columns(x, cross(z, x), z), center);
}

function armFrame(anchor, direction, axis) {
  const z = unit(axis, 'arm axis');
  const x = unit(sub(direction, scale(z, dot(z, direction))), 'arm direction');
  return frame(columns(x, cross(z, x), z), anchor);
}

function rodFrame(start, end) {
  const x = unit(sub(end, start), 'rod direction');
  let z = [0, 0, 1];
  if (Math.abs(dot(x, z)) > 0.95) z = [0, 1, 0];
  z = unit(sub(z, scale(x, dot(z, x))), 'rod normal');
  return frame(columns(x, cross(z, x), z), start);
}

function setFrame(poses, name, value) {
  if (!EXPECTED_NAMES.has(name)) throw new Error(`Unknown C2 mesh name: ${name}`);
  poses.set(name, value);
}

function setRod(poses, name, start, end) {
  setFrame(poses, name, rodFrame(start, end));
}

function setPin(poses, name, center, axis) {
  const value = axisFrame(center, axis);
  setFrame(poses, name, value);
  setFrame(poses, `${name}Head`, value);
  setFrame(poses, `${name}Retainer`, value);
}

function setSpherical(poses, prefix, center, axis, line, bracket) {
  const u = unit(axis, `${prefix} axis`);
  const v = unit(line, `${prefix} line`);
  const pin = axisFrame(center, u);
  const outerAxis = unit(sub(u, scale(v, dot(u, v))), `${prefix} eye axis`);
  const eye = axisFrame(center, outerAxis, v);
  const fork = axisFrame(center, u, bracket);
  setFrame(poses, `${prefix}Ball`, pin);
  setFrame(poses, `${prefix}Eye`, eye);
  setFrame(poses, `${prefix}SpacerL`, axisFrame(add(center, scale(u, -31)), u));
  setFrame(poses, `${prefix}SpacerR`, axisFrame(add(center, scale(u, 31)), u));
  setFrame(poses, `${prefix}Fork`, fork);
  setPin(poses, `${prefix}Pin`, center, u);
}

function addScissor(poses, prefix, scissors, rotation, psi, center, rotating, bracketLengths) {
  const base = vector(scissors.base, `${prefix} base`);
  const elbow = vector(scissors.elbow, `${prefix} elbow`);
  const end = vector(scissors.end, `${prefix} end`);
  const axis = unit(vector(scissors.base_axis, `${prefix} base axis`), `${prefix} base axis`);
  const radial = unit(vector(scissors.radial_basis, `${prefix} radial basis`), `${prefix} radial basis`);
  setFrame(poses, `${prefix}ArmA`, armFrame(base, sub(elbow, base), axis));
  setFrame(poses, `${prefix}ArmB`, armFrame(elbow, sub(end, elbow), axis));
  setPin(poses, `${prefix}BasePin`, base, axis);
  setPin(poses, `${prefix}ElbowPin`, elbow, axis);
  setFrame(poses, `${prefix}BaseBearing`, axisFrame(base, axis));
  setSpherical(poses, `${prefix}End`, end, column(rotation, 1), sub(elbow, end), scale(radial, -1));

  if (rotating) {
    const mast = rz(deg(psi));
    const start = multiplyVec3(mast, [150, 0, 1900]);
    setRod(poses, `${prefix}Support`, start, sub(base, scale(column(mast, 0), 35)));
  } else {
    setRod(poses, `${prefix}Support`, sub(base, [0, 0, 360]), sub(base, [0, 0, 35]));
  }
  // RingBracket endpoints change with the solved scissors.  Keep its neutral
  // CAD length and aim it at the current endpoint; a variable-length rod is
  // not a rigid transform and would stretch the source mesh.
  const ringStart = sub(end, scale(radial, 75));
  const ringEnd = add(center, multiplyVec3(rotation, [710, 0, rotating ? 10 : -100]));
  bracketLengths[prefix] = norm(sub(ringEnd, ringStart));
  setRod(poses, `${prefix}RingBracket`, ringStart, ringEnd);
}

function buildPose(collective, cyclicC, cyclicS, azimuth) {
  const frames = swashFrames(collective, cyclicC, cyclicS, azimuth);
  const center = vector(frames.center, 'swash center');
  const qs = matrix(frames.stator_rotation, 'stator rotation');
  const qr = matrix(frames.rotor_rotation, 'rotor rotation');
  const poses = new Map(Array.from(EXPECTED_NAMES, name => [name, identityFrame()]));
  const bracketLengths = {};

  // Mast and hub primitives are rotor-side ownership in the live rig.  Most
  // are axial-symmetric, while MastFlange's index holes make the yaw visible;
  // the saved builder leaves these primitives in global axes for its snapshots.
  const hubFrame = frame(rz(deg(azimuth)), [0, 0, 0]);
  for (const name of ['Mast', 'MastFlange', 'HubRing', 'HubCap']) setFrame(poses, name, hubFrame);

  setFrame(poses, 'GuideBushM145', frame(IDENTITY, center));
  setFrame(poses, 'GuideBush100', frame(IDENTITY, center));
  setFrame(poses, 'SliderSphere', frame(IDENTITY, center));
  for (const name of ['StatorSocket', 'SocketSplitBolts', 'LowerRace', 'BearingBalls', 'BearingCage', 'LowerRaceBolts']) {
    setFrame(poses, name, frame(qs, center));
  }
  for (const name of ['UpperRace', 'RotorRing', 'UpperRaceBolts']) {
    setFrame(poses, name, frame(qr, center));
  }

  const servoAngles = [30, 150, 270];
  for (let index = 0; index < 3; index += 1) {
    const ar = rz(deg(servoAngles[index]));
    const er = column(ar, 0);
    const tangent = column(ar, 1);
    const bottom = add(scale(er, 710), [0, 0, 150]);
    const top = add(center, multiplyVec3(qs, add(scale(er, 710), [0, 0, -190])));
    const line = unit(sub(top, bottom), `Servo${index} line`);
    const prefix = `Servo${index}`;
    setSpherical(poses, `${prefix}Lower`, bottom, tangent, line, [0, 0, -1]);
    setSpherical(poses, `${prefix}Upper`, top, multiplyVec3(qs, tangent), scale(line, -1), column(qs, 2));
    setRod(poses, `${prefix}Body`, add(bottom, scale(line, 80)), add(bottom, scale(line, 500)));
    setRod(poses, `${prefix}Rod`, add(top, scale(line, -580)), add(top, scale(line, -80)));
    setFrame(poses, `${prefix}Piston`, axisFrame(add(top, scale(line, -570)), line));
  }

  const stator = frames.stator_scissors;
  const rotor = frames.rotor_scissors;
  addScissor(poses, 'Antirotation', stator, qs, azimuth, center, false, bracketLengths);
  addScissor(poses, 'RotatingDrive', rotor, qr, azimuth, center, true, bracketLengths);

  const blades = [];
  for (let index = 0; index < 5; index += 1) {
    const angle = azimuth + index * 72;
    const ar = rz(deg(angle));
    const state = solve(angle, collective, cyclicC, cyclicS, 0, 0, azimuth);
    const rot = matrix(state.rotation, `Blade${index + 1} rotation`);
    const pivot = vector(state.pivot, `Blade${index + 1} pivot`);
    const top = vector(state.top, `Blade${index + 1} top`);
    const bottom = vector(state.bottom, `Blade${index + 1} bottom`);
    const line = unit(sub(top, bottom), `Blade${index + 1} pitch link`);
    const prefix = `Blade${index + 1}`;
    const housingFrame = frame(ar, [0, 0, HUB_Z]);
    const arPivotFrame = frame(ar, pivot);
    const cuffFrame = frame(rot, pivot);
    setFrame(poses, `${prefix}Housing`, housingFrame);
    for (const name of ['OuterRace', 'Elastomer', 'Shims', 'StopCradle', 'LagStopM1', 'LagStop1', 'ParkShoe']) {
      setFrame(poses, `${prefix}${name}`, arPivotFrame);
    }
    for (const name of ['InnerRace', 'CuffHorn', 'FoldClevis']) {
      setFrame(poses, `${prefix}${name}`, cuffFrame);
    }

    const localBladeOffset = rz(deg(index * 72));
    const lowerAxis = multiplyVec3(qr, column(localBladeOffset, 0));
    setSpherical(poses, `${prefix}PitchLower`, bottom, lowerAxis, line, scale(column(qr, 2), -1));
    setSpherical(poses, `${prefix}PitchUpper`, top, column(rot, 0), scale(line, -1), column(rot, 1));
    setRod(poses, `${prefix}PitchLink`, bottom, top);

    const da = add(multiplyVec3(ar, [420, -400, 0]), [0, 0, HUB_Z]);
    const db = add(pivot, multiplyVec3(rot, [450, -350, 0]));
    const damperLine = unit(sub(db, da), `${prefix} damper line`);
    setSpherical(poses, `${prefix}DamperA`, da, column(ar, 2), damperLine, column(ar, 1));
    setSpherical(poses, `${prefix}DamperB`, db, column(rot, 2), scale(damperLine, -1), column(rot, 1));
    setFrame(poses, `${prefix}DamperMount`, housingFrame);
    setFrame(poses, `${prefix}DamperLug`, cuffFrame);
    setRod(poses, `${prefix}DamperBody`, add(da, scale(damperLine, 80)), add(da, scale(damperLine, 350)));
    setRod(poses, `${prefix}DamperRod`, add(db, scale(damperLine, -430)), add(db, scale(damperLine, -80)));
    setFrame(poses, `${prefix}DamperPiston`, axisFrame(add(db, scale(damperLine, -420)), damperLine));

    const origin = add(pivot, multiplyVec3(rot, [FOLD_PIN, 0, 0]));
    const lockCenter = add(pivot, multiplyVec3(rot, [FOLD_PIN - 200, 0, 0]));
    for (const name of ['FoldTang', 'Airfoil', 'RootSpar']) setFrame(poses, `${prefix}${name}`, frame(rot, origin));
    setPin(poses, `${prefix}FoldHinge`, origin, column(rot, 2));
    setPin(poses, `${prefix}FlightLock`, lockCenter, column(rot, 2));
    for (const name of ['LockGuide', 'LockWebM1', 'LockWeb1']) setFrame(poses, `${prefix}${name}`, cuffFrame);
    blades.push(state);
  }
  return {frames, poses, blades, bracketLengths};
}

function deltaMatrix(current, neutral) {
  const r = multiply3(current.r, transpose(neutral.r));
  const t = sub(current.t, multiplyVec3(r, neutral.t)).map(value => value / 1000);
  return new THREE.Matrix4().set(
    r[0], r[1], r[2], t[0],
    r[3], r[4], r[5], t[1],
    r[6], r[7], r[8], t[2],
    0, 0, 0, 1,
  );
}

function collectMeshes(group) {
  if (!group || typeof group.traverse !== 'function') throw new TypeError('createRig requires a Three.js Group');
  const meshes = new Map();
  group.traverse(object => {
    if (!object.isMesh && object.type !== 'Mesh') return;
    const name = String(object.name || '');
    if (!EXPECTED_NAMES.has(name)) throw new Error(`Unknown C2 mesh name: ${name || '<unnamed>'}`);
    if (meshes.has(name)) throw new Error(`Duplicate C2 mesh name: ${name}`);
    meshes.set(name, object);
  });
  for (const name of EXPECTED_NAMES) {
    if (!meshes.has(name)) throw new Error(`Missing C2 mesh name: ${name}`);
  }
  return meshes;
}

/**
 * Attach C2 rigid kinematics to the 425 neutral CAD meshes.
 * Mesh vertices stay in their exported metre/world coordinates; only each
 * object's matrix changes. Folding, flap, and lag are intentionally outside
 * this flight-control rig and remain represented by the saved CAD snapshots.
 */
export function createRig(group) {
  const meshes = collectMeshes(group);
  const neutral = buildPose(12, 0, 0, 0);
  let last = null;

  function update({collective = 12, cyclicC = 0, cyclicS = 0, azimuth = 0} = {}) {
    const pose = buildPose(collective, cyclicC, cyclicS, azimuth);
    for (const [name, object] of meshes) {
      const current = pose.poses.get(name);
      object.matrixAutoUpdate = false;
      object.matrix.copy(deltaMatrix(current, neutral.poses.get(name)));
      object.matrixWorldNeedsUpdate = true;
    }
    if (typeof group.updateMatrixWorld === 'function') group.updateMatrixWorld(true);
    last = pose;
    const statorError = Math.abs(pose.bracketLengths.Antirotation - neutral.bracketLengths.Antirotation);
    const rotorError = Math.abs(pose.bracketLengths.RotatingDrive - neutral.bracketLengths.RotatingDrive);
    return {
      blades: pose.blades,
      frames: pose.frames,
      ringBracketApproximation: {
        stator_mm: statorError,
        rotor_mm: rotorError,
        max_mm: Math.max(statorError, rotorError),
        sampled_limit_mm: RING_BRACKET_APPROXIMATION_SAMPLE_MAX_MM,
      },
    };
  }

  return {
    update,
    get last() { return last; },
  };
}

export {EXPECTED_NAMES};
