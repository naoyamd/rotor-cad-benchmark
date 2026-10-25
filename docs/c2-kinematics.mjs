// Dependency-free C2 continuous kinematics.  Lengths are millimetres and
// public angles are degrees; matrices are row-major arrays.

import {
  P as C1P,
  deg,
  multiply3,
  multiplyVec3,
  rx,
  ry,
  rz,
} from './kinematics.mjs';

const X = [1, 0, 0];
const Y = [0, 1, 0];
const Z = [0, 0, 1];

export const P = {
  ...C1P,
  stator_base: [900, 0, 350],
  rotor_base: [350, 0, 1900],
  stator_end_offset: [820, 0, -150],
  rotor_end_offset: [820, 0, 70],
  scissor_arm_lengths: [550, 510],
  stator_elbow_sign: 1,
  rotor_elbow_sign: -1,
  stator_joint_limits_deg: [[-180, 180], [-180, 180]],
  rotor_joint_limits_deg: [[-180, 180], [-180, 180]],
  scissor_yaw_limit_deg: 35,
  pitch_limits_deg: [-35, 45],
  scissors_tolerance_mm: 1e-7,
};

const add3 = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const subtract3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm3 = a => Math.hypot(a[0], a[1], a[2]);
const cross3 = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const scale3 = (a, scale) => a.map(value => value * scale);

function vector(value, name) {
  if (!Array.isArray(value) || value.length !== 3 || value.some(component => !Number.isFinite(component))) {
    throw new TypeError(`${name} must be a finite 3-vector`);
  }
  return value.map(Number);
}

function unit(value, name) {
  const result = vector(value, name);
  const length = norm3(result);
  if (length <= 0) throw new RangeError(`${name} must be non-zero`);
  return scale3(result, 1 / length);
}

function wrapNear(angle, reference) {
  return angle + 2 * Math.PI * Math.floor((reference - angle + Math.PI) / (2 * Math.PI));
}

function within(value, limits, tolerance = 1e-9) {
  return limits[0] - tolerance <= value && value <= limits[1] + tolerance;
}

function validateControls(collective, cyclicC, cyclicS, flap, lag) {
  for (const [name, value] of Object.entries({collective, cyclicC, cyclicS, flap, lag})) {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
  }
  if (collective < 8 || collective > 16) {
    throw new RangeError('collective must be between 8 and 16 degrees');
  }
  if (Math.hypot(cyclicC, cyclicS) > 3 + 1e-12) {
    throw new RangeError('cyclic magnitude must be at most 3 degrees');
  }
  if (flap < -3 || flap > 6) throw new RangeError('flap must be between -3 and 6 degrees');
  if (Math.abs(lag) > 3) throw new RangeError('lag must be within +/-3 degrees');
}

export function swash(collective = 12, cyclicC = 0, cyclicS = 0, p = P) {
  validateControls(collective, cyclicC, cyclicS, 0, 0);
  const h = p.horn * (Math.sin(deg(collective)) - Math.sin(deg(p.collective_ref)));
  const phase = Math.atan2(p.horn, p.horn_x);
  const scale = p.horn * Math.cos(deg(p.collective_ref)) / Math.hypot(p.horn_x, p.horn);
  const c = deg(cyclicC) * scale;
  const s = deg(cyclicS) * scale;
  const gx = c * Math.cos(phase) - s * Math.sin(phase);
  const gy = c * Math.sin(phase) + s * Math.cos(phase);
  const n = [-gx, -gy, 1];
  const nScale = norm3(n);
  for (let i = 0; i < 3; i++) n[i] /= nScale;
  const v = cross3(Z, n);
  const k = [0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0];
  const kk = multiply3(k, k);
  const q = kk.map((value, i) => (i % 4 === 0 ? 1 : 0) + k[i] + value / (1 + n[2]));
  return {center: [0, 0, p.swash_z + h], q};
}

function solveSinusoid(a, b, c, reference, limit, name) {
  const radius = Math.hypot(a, b);
  if (radius <= 1e-14) {
    if (Math.abs(c) <= 1e-10) return reference;
    throw new Error(`No finite ${name} closure`);
  }
  const ratio = -c / radius;
  if (Math.abs(ratio) > 1 + 1e-12) throw new Error(`No finite ${name} closure`);
  const phase = Math.atan2(b, a);
  const offset = Math.acos(Math.max(-1, Math.min(1, ratio)));
  const candidates = [phase - offset, phase + offset]
    .map(candidate => wrapNear(candidate, reference))
    .filter(candidate => within(candidate - reference, [-limit, limit], 1e-10));
  if (!candidates.length) throw new Error(`${name} branch outside yaw limit`);
  return candidates.reduce((best, candidate) => (
    Math.abs(candidate - reference) < Math.abs(best - reference) ? candidate : best
  ));
}

function planarRrs(baseValue, endValue, radialValue, axisValue, armLengths, jointLimits, name, elbowSign = 1) {
  const base = vector(baseValue, `${name} base`);
  const end = vector(endValue, `${name} end`);
  const radial = unit(radialValue, `${name} radial basis`);
  const axis = unit(axisValue, `${name} revolute axis`);
  let zAxis = unit(cross3(axis, radial), `${name} plane basis`);
  if (dot3(zAxis, Z) < 0) zAxis = scale3(zAxis, -1);
  const lengths = armLengths.map(Number);
  if (lengths.length !== 2 || lengths.some(value => value <= 0 || !Number.isFinite(value))) {
    throw new RangeError(`${name} arm lengths must be positive`);
  }
  const limits = jointLimits.map(limit => limit.map(Number));
  if (limits.length !== 2 || limits.some(limit => limit.length !== 2 || limit[0] > limit[1])) {
    throw new RangeError(`${name} joint limits are invalid`);
  }

  const d = subtract3(end, base);
  const tangentialResidual = dot3(d, axis);
  if (Math.abs(tangentialResidual) > 1e-6) throw new Error(`${name} endpoint is outside its revolute plane`);
  const x = dot3(d, radial);
  const z = dot3(d, zAxis);
  const [l1, l2] = lengths;
  const radiusSq = x * x + z * z;
  const cosQ2 = (radiusSq - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  if (cosQ2 < -1 - 1e-10 || cosQ2 > 1 + 1e-10) throw new Error(`No finite ${name} arm closure`);
  const q2 = elbowSign * Math.acos(Math.max(-1, Math.min(1, cosQ2)));
  const q1 = Math.atan2(z, x) - Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));
  const angles = [q1, q2];
  const anglesDeg = angles.map(value => value / (Math.PI / 180));
  if (!anglesDeg.every((value, index) => within(value, limits[index]))) {
    throw new Error(`${name} joint angle outside limit`);
  }
  const elbow = add3(base, add3(scale3(radial, l1 * Math.cos(q1)), scale3(zAxis, l1 * Math.sin(q1))));
  const first = subtract3(elbow, base);
  const second = subtract3(end, elbow);
  const closure = Math.max(Math.abs(norm3(first) - l1), Math.abs(norm3(second) - l2));
  const radialProjection = [dot3(base, radial), dot3(elbow, radial), dot3(end, radial)];
  const radii = [base, elbow, end].map(point => Math.hypot(point[0], point[1]));
  return {
    base,
    elbow,
    end,
    joint_1: elbow,
    joint_2: end,
    endpoint: end,
    axes: [axis, axis],
    base_axis: axis,
    elbow_axis: axis,
    radial_basis: radial,
    z_basis: zAxis,
    arm_lengths_mm: [l1, l2],
    joint_angles_rad: angles,
    joint_angles_deg: anglesDeg,
    joint_limits_deg: limits,
    radial_projection_mm: radialProjection,
    radii_mm: radii,
    min_radius_mm: Math.min(...radii),
    min_radial_projection_mm: Math.min(...radialProjection),
    planar_residual_mm: Math.abs(tangentialResidual),
    closure_residual_mm: closure,
    valid: closure <= 1e-7 && Math.abs(tangentialResidual) <= 1e-6,
  };
}

export function swashFrames(collective = 12, cc = 0, cs = 0, psi = 0, p = P) {
  validateControls(collective, cc, cs, 0, 0);
  if (!Number.isFinite(psi)) throw new TypeError('psi must be finite');
  const psiRad = deg(psi);
  const {center, q} = swash(collective, cc, cs, p);
  const endStator = vector(p.stator_end_offset, 'stator end offset');
  const endRotor = vector(p.rotor_end_offset, 'rotor end offset');
  const yawLimit = deg(p.scissor_yaw_limit_deg);
  if (yawLimit <= 0 || !Number.isFinite(yawLimit)) throw new RangeError('scissor_yaw_limit_deg must be positive');

  const aS = q[3] * endStator[0];
  const bS = q[4] * endStator[0];
  const cS = center[1] + q[5] * endStator[2];
  const alpha = solveSinusoid(aS, bS, cS, 0, yawLimit, 'stator yaw');
  const statorRotation = multiply3(q, rz(alpha));
  const statorEnd = add3(center, multiplyVec3(statorRotation, endStator));
  const stator = planarRrs(
    p.stator_base,
    statorEnd,
    X,
    Y,
    p.scissor_arm_lengths,
    p.stator_joint_limits_deg,
    'stator scissor',
    Number(p.stator_elbow_sign),
  );

  const mast = rz(psiRad);
  const rotorBase = multiplyVec3(mast, vector(p.rotor_base, 'rotor base'));
  const rotorRadial = multiplyVec3(mast, X);
  const rotorAxis = multiplyVec3(mast, Y);
  const rowT = [dot3(rotorAxis, [q[0], q[3], q[6]]), dot3(rotorAxis, [q[1], q[4], q[7]]), dot3(rotorAxis, [q[2], q[5], q[8]])];
  const aR = rowT[0] * endRotor[0];
  const bR = rowT[1] * endRotor[0];
  const cR = dot3(rotorAxis, center) + rowT[2] * endRotor[2] - dot3(rotorAxis, rotorBase);
  const beta = solveSinusoid(aR, bR, cR, psiRad, yawLimit, 'rotor yaw');
  const rotorRotation = multiply3(q, rz(beta));
  const rotorEnd = add3(center, multiplyVec3(rotorRotation, endRotor));
  const rotor = planarRrs(
    rotorBase,
    rotorEnd,
    rotorRadial,
    rotorAxis,
    p.scissor_arm_lengths,
    p.rotor_joint_limits_deg,
    'rotor scissor',
    Number(p.rotor_elbow_sign),
  );

  return {
    center,
    q,
    stator_rotation: statorRotation,
    rotor_rotation: rotorRotation,
    alpha_rad: alpha,
    alpha_deg: alpha / (Math.PI / 180),
    beta_rad: beta,
    beta_deg: beta / (Math.PI / 180),
    alpha,
    beta,
    stator_end: statorEnd,
    rotor_end: rotorEnd,
    stator_endpoint: statorEnd,
    rotor_endpoint: rotorEnd,
    scissors_endpoints: {stator: statorEnd, rotor: rotorEnd},
    stator_revolute_axes: stator.axes,
    rotor_revolute_axes: rotor.axes,
    stator_scissors: stator,
    rotor_scissors: rotor,
    stator,
    rotor,
    stator_plane_residual_mm: Math.abs(statorEnd[1]),
    rotor_plane_residual_mm: Math.abs(dot3(subtract3(rotorEnd, rotorBase), rotorAxis)),
  };
}

function pitchCandidates(base, bottom, articulated, p, reference) {
  const horn = Number(p.horn);
  const d = subtract3(base, bottom);
  const u = multiplyVec3(articulated, [0, horn, 0]);
  const v = multiplyVec3(articulated, [0, 0, horn]);
  const a = 2 * dot3(d, u);
  const b = 2 * dot3(d, v);
  const c = Number(p.link_length) ** 2 - dot3(d, d) - horn ** 2;
  const amplitude = Math.hypot(a, b);
  if (Math.abs(c) > amplitude + 1e-10) throw new Error('No finite-link closure');
  if (amplitude <= 1e-14) throw new Error('Pitch linkage is singular');
  const delta = Math.atan2(b, a);
  const ac = Math.acos(Math.max(-1, Math.min(1, c / amplitude)));
  const limits = p.pitch_limits_deg.map(Number);
  const candidates = [delta - ac, delta + ac]
    .map(value => wrapNear(value, deg(p.collective_ref ?? 12)))
    .filter(value => within(value / (Math.PI / 180), limits));
  if (!candidates.length) throw new Error('Pitch outside assembly branch');
  return {
    theta: candidates.reduce((best, value) => Math.abs(value - deg(reference)) < Math.abs(best - deg(reference)) ? value : best),
    u,
    v,
    a,
    b,
  };
}

export function solve(
  psi,
  collective = 12,
  cyclicC = 0,
  cyclicS = 0,
  flap = 0,
  lag = 0,
  mastPsi = psi,
  p = P,
) {
  if (!Number.isFinite(psi)) throw new TypeError('psi must be finite');
  if (!Number.isFinite(mastPsi)) throw new TypeError('mastPsi must be finite');
  validateControls(collective, cyclicC, cyclicS, flap, lag);
  const frames = swashFrames(collective, cyclicC, cyclicS, mastPsi, p);
  const bladeOffset = deg(psi - mastPsi);
  const rotorHorn = [p.horn_x, p.horn, p.swash_joint_z];
  const bottom = add3(frames.center, multiplyVec3(frames.rotor_rotation, multiplyVec3(rz(bladeOffset), rotorHorn)));

  const az = rz(deg(psi));
  const articulated = multiply3(multiply3(az, rz(deg(lag))), ry(-deg(flap)));
  const pivot = add3(multiplyVec3(az, [p.joint, 0, 0]), [0, 0, p.hub_z]);
  const base = add3(pivot, multiplyVec3(articulated, [p.horn_x - p.joint, 0, 0]));
  const {theta, u, v, a, b} = pitchCandidates(base, bottom, articulated, p, collective);
  const top = add3(base, add3(scale3(u, Math.cos(theta)), scale3(v, Math.sin(theta))));
  const sensitivity = Math.abs((-a * Math.sin(theta) + b * Math.cos(theta)) / (2 * p.link_length));
  const actuatorLength = norm3(subtract3(top, bottom));
  const residual = Math.abs(actuatorLength - p.link_length);
  const pitchDeg = theta / (Math.PI / 180);
  return {
    theta: pitchDeg,
    top,
    bottom,
    pivot,
    rotation: multiply3(articulated, rx(theta)),
    residual,
    lever: sensitivity,
    pitch_deg: pitchDeg,
    blade_azimuth_deg: psi,
    mast_azimuth_deg: mastPsi,
    blade_offset_deg: psi - mastPsi,
    center: frames.center,
    stator_rotation: frames.stator_rotation,
    rotor_rotation: frames.rotor_rotation,
    stator_scissors: frames.stator_scissors,
    rotor_scissors: frames.rotor_scissors,
    swash_frames: frames,
    actuator_length_mm: actuatorLength,
    servo_pin_axis: multiplyVec3(az, Y),
    pitch_pin_axis: multiplyVec3(az, X),
  };
}

