import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { test } from 'node:test';
import { P, solve, swash } from '../kinematics.mjs';

const evidence = JSON.parse(await readFile(
  new URL('../../results/C1/verification/numerical.json', import.meta.url),
));

const close = (actual, expected, tolerance, message) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${message}: ${actual} vs ${expected}`);
};

test('neutral solve agrees with the Python model geometry', () => {
  const state = solve(0);
  close(state.theta, 12, 1e-12, 'neutral pitch');
  close(state.residual, 0, 1e-9, 'neutral closure');
  close(state.top[0], 600, 1e-9, 'neutral top x');
  close(state.top[1], 244.5369001834514, 1e-9, 'neutral top y');
  close(state.top[2], 2251.97792270444, 1e-9, 'neutral top z');
});

test('swash row-major rotation and cyclic response match evidence', () => {
  const state = swash(12, 3, 0);
  close(state.center[2], 1100, 1e-12, 'neutral swash height');
  assert.equal(state.q.length, 9);
  assert.deepEqual(state.q.map(Number.isFinite), Array(9).fill(true));
  const pitches = [0, 72, 144, 216, 288].map(psi => solve(psi, 12, 0.01, 0).theta);
  const expected = evidence.kinematics.summary.jacobian.map(row => row[1]);
  for (let i = 0; i < pitches.length; i += 1) close(
    (pitches[i] - solve(i * 72).theta) / 0.01,
    expected[i],
    2e-5,
    `cyclic-c Jacobian row ${i}`,
  );
});

test('grid closures remain at numerical zero', () => {
  let maxResidual = 0;
  for (let psi = 0; psi < 360; psi += 12) {
    for (const collective of [8, 12, 16]) {
      for (let azimuth = 0; azimuth < 360; azimuth += 45) {
        const cyclicC = 3 * Math.cos(azimuth * Math.PI / 180);
        const cyclicS = 3 * Math.sin(azimuth * Math.PI / 180);
        for (const [flap, lag] of [[0, 0], [-3, -3], [-3, 3], [6, -3], [6, 3]]) {
          maxResidual = Math.max(maxResidual, solve(psi, collective, cyclicC, cyclicS, flap, lag).residual);
        }
      }
    }
  }
  close(maxResidual, evidence.kinematics['12deg_grid'].residual_mm, 1e-9, 'grid residual');
});

test('unsupported or non-finite controls are rejected at the boundary', () => {
  for (const args of [
    [0, 7], [0, 17], [0, 12, 3.01, 0], [0, 12, 0, 0, -3.01, 0],
    [0, 12, 0, 0, 6.01, 0], [0, 12, 0, 0, 0, 3.01], [NaN], [Infinity],
  ]) assert.throws(() => solve(...args), /finite|between|magnitude|within/);
  assert.throws(() => solve(0, 12, 3, 3), /magnitude/);
  assert.equal(P.units, 'mm');
});

