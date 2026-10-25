import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {test} from 'node:test';
import {solve, swashFrames} from '../c2-kinematics.mjs';

const fixture = JSON.parse(await readFile(
  new URL('./fixtures/c2-kinematics.json', import.meta.url),
));

function close(actual, expected, tolerance, path) {
  assert.equal(typeof actual, 'number', `${path} is not numeric`);
  assert.ok(Number.isFinite(actual), `${path} is not finite`);
  assert.ok(Math.abs(actual - expected) <= tolerance, `${path}: ${actual} vs ${expected}`);
}

function closeShape(actual, expected, tolerance, path) {
  if (Array.isArray(expected)) {
    assert.ok(Array.isArray(actual), `${path} is not an array`);
    assert.equal(actual.length, expected.length, `${path} length`);
    expected.forEach((value, index) => closeShape(actual[index], value, tolerance, `${path}[${index}]`));
    return;
  }
  if (expected && typeof expected === 'object') {
    assert.ok(actual && typeof actual === 'object', `${path} is not an object`);
    for (const [key, value] of Object.entries(expected)) {
      closeShape(actual[key], value, tolerance, `${path}.${key}`);
    }
    return;
  }
  close(actual, expected, tolerance, path);
}

function controls(input) {
  return [input.collective, input.cc, input.cs, input.flap, input.lag];
}

test('C2 continuous solver agrees with mechanism.py fixtures', () => {
  assert.equal(fixture.source, 'results/C2/src/mechanism.py');
  for (const entry of fixture.cases) {
    const input = entry.input;
    const frames = swashFrames(input.collective, input.cc, input.cs, input.mastPsi);
    const state = solve(input.psi, ...controls(input), input.mastPsi);
    for (const [key, expected] of Object.entries(entry.frames)) {
      closeShape(frames[key], expected, 1e-8, `${input.name}.frames.${key}`);
    }
    for (const [key, expected] of Object.entries(entry.solve)) {
      closeShape(state[key], expected, 1e-8, `${input.name}.solve.${key}`);
    }
  }
});

test('mast phase is applied to all five blade solves', () => {
  const input = fixture.five_blades.input;
  fixture.five_blades.solves.forEach((expected, index) => {
    const state = solve(input.mastPsi + index * 72, ...controls(input), input.mastPsi);
    for (const [key, value] of Object.entries(expected)) {
      closeShape(state[key], value, 1e-8, `blade${index}.${key}`);
    }
  });
});

test('yaw branches remain continuous across the 360-degree wrap', () => {
  const args = [15.5, 1.75, -1.4, 4, -2.25, 17.5];
  const low = solve(-0.001, ...args);
  const wrapped = solve(359.999, ...args);
  for (const key of ['theta', 'top', 'bottom', 'pivot', 'rotation', 'residual']) {
    closeShape(wrapped[key], low[key], 1e-8, `wrap.${key}`);
  }
  const lowFrames = swashFrames(args[0], args[1], args[2], -0.001);
  const wrappedFrames = swashFrames(args[0], args[1], args[2], 359.999);
  for (const key of ['center', 'q', 'stator_rotation', 'rotor_rotation', 'stator_scissors', 'rotor_scissors']) {
    if (key.endsWith('scissors')) {
      for (const point of ['base', 'elbow', 'end', 'base_axis', 'elbow_axis', 'radial_basis', 'z_basis']) {
        closeShape(wrappedFrames[key][point], lowFrames[key][point], 1e-8, `wrap.frames.${key}.${point}`);
      }
    } else {
      closeShape(wrappedFrames[key], lowFrames[key], 1e-8, `wrap.frames.${key}`);
    }
  }
});

test('supported controls reject non-finite and out-of-range input', () => {
  assert.throws(() => swashFrames(NaN), /finite/);
  assert.throws(() => solve(Infinity), /finite/);
  assert.throws(() => solve(0, 7), /collective/);
  assert.throws(() => solve(0, 17), /collective/);
  assert.throws(() => solve(0, 12, 3, 1), /cyclic/);
  assert.throws(() => solve(0, 12, 0, 0, -3.1), /flap/);
  assert.throws(() => solve(0, 12, 0, 0, 6.1), /flap/);
  assert.throws(() => solve(0, 12, 0, 0, 0, -3.1), /lag/);
  assert.throws(() => solve(0, 12, 0, 0, 0, 3.1), /lag/);
  assert.throws(() => solve(0, 12, 0, 0, 0, 0, NaN), /mastPsi/);
});
