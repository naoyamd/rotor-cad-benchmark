// Dependency-free port of results/C1/src/mechanism.py's swash and solve.
// All lengths in this module are millimetres; angles are degrees at the API.

const RAD = Math.PI / 180;

export const P = {
  units: 'mm',
  radius: 9300,
  chord: 650,
  root: 1600,
  joint: 600,
  hub_z: 2200,
  swash_z: 1100,
  swash_joint_z: 100,
  horn_x: 600,
  horn: 250,
  collective_ref: 12,
  rpm: 210,
  blade_mass: 180,
  aircraft_mass: 15600,
  rho: 1.225,
  fold_pin: 1500,
  link_length: Math.hypot(
    250 * (Math.cos(12 * RAD) - 1),
    1000 + 250 * Math.sin(12 * RAD),
  ),
};

export const deg = angle => angle * RAD;

export function rx(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [1, 0, 0, 0, c, -s, 0, s, c];
}

export function ry(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [c, 0, s, 0, 1, 0, -s, 0, c];
}

export function rz(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

export function multiply3(a, b) {
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

export function multiplyVec3(a, v) {
  return [
    a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
    a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
    a[6] * v[0] + a[7] * v[1] + a[8] * v[2],
  ];
}

const add3 = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const subtract3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm3 = a => Math.hypot(a[0], a[1], a[2]);
const cross3 = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

function validateControls(collective, cyclicC, cyclicS, flap, lag) {
  for (const [name, value] of Object.entries({ collective, cyclicC, cyclicS, flap, lag })) {
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
  const n0 = [-gx, -gy, 1];
  const nScale = norm3(n0);
  const n = n0.map(value => value / nScale);
  const v = cross3([0, 0, 1], n);
  const k = [0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0];
  const kk = multiply3(k, k);
  const q = kk.map((value, i) => (
    (i % 4 === 0 ? 1 : 0) + k[i] + value / (1 + n[2])
  ));
  return { center: [0, 0, p.swash_z + h], q };
}

export function solve(
  psi,
  collective = 12,
  cyclicC = 0,
  cyclicS = 0,
  flap = 0,
  lag = 0,
  p = P,
) {
  if (!Number.isFinite(psi)) throw new TypeError('psi must be finite');
  validateControls(collective, cyclicC, cyclicS, flap, lag);

  const { center, q } = swash(collective, cyclicC, cyclicS, p);
  const az = rz(deg(psi));
  const articulated = multiply3(multiply3(az, rz(deg(lag))), ry(-deg(flap)));
  const pivot = add3(multiplyVec3(az, [p.joint, 0, 0]), [0, 0, p.hub_z]);
  const bottom = add3(
    center,
    multiplyVec3(q, multiplyVec3(az, [p.horn_x, p.horn, p.swash_joint_z])),
  );
  const base = add3(
    pivot,
    multiplyVec3(articulated, [p.horn_x - p.joint, 0, 0]),
  );
  const u = multiplyVec3(articulated, [0, p.horn, 0]);
  const v = multiplyVec3(articulated, [0, 0, p.horn]);
  const d = subtract3(base, bottom);
  const a = 2 * dot3(d, u);
  const b = 2 * dot3(d, v);
  const c = p.link_length ** 2 - dot3(d, d) - p.horn ** 2;
  const amplitude = Math.hypot(a, b);
  if (Math.abs(c) > amplitude) throw new Error('No finite-link closure');

  const delta = Math.atan2(b, a);
  const acosArg = Math.max(-1, Math.min(1, c / amplitude));
  const ac = Math.acos(acosArg);
  const candidates = [delta - ac, delta + ac, delta - ac - 2 * Math.PI, delta + ac - 2 * Math.PI]
    .filter(theta => -deg(35) < theta && theta < deg(45));
  if (!candidates.length) throw new Error('Pitch outside assembly branch');
  const theta = candidates.reduce((best, candidate) => (
    Math.abs(candidate - deg(collective)) < Math.abs(best - deg(collective)) ? candidate : best
  ));
  const top = add3(base, add3(u.map(value => value * Math.cos(theta)), v.map(value => value * Math.sin(theta))));
  const lever = Math.abs((-a * Math.sin(theta) + b * Math.cos(theta)) / (2 * p.link_length));
  const residual = Math.abs(norm3(subtract3(top, bottom)) - p.link_length);
  return {
    theta: theta / RAD,
    top,
    bottom,
    pivot,
    rotation: multiply3(articulated, rx(theta)),
    residual,
    lever,
  };
}

