import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile, stat} from 'node:fs/promises';
import {createHash} from 'node:crypto';

const states = ['neutral', 'max-input', 'parked', 'folded'];
const readJson = path => readFile(new URL(path, import.meta.url), 'utf8').then(JSON.parse);

for (const state of states) {
  test(`C2 ${state}: 425 source-traced meshes have valid indices`, async () => {
    const asset = await readJson(`../assets/c2/${state}.json`);
    const source = await readFile(new URL(`../../results/C2/rotor-${state}.FCStd`, import.meta.url));
    assert.equal(asset.source_sha256, createHash('sha256').update(source).digest('hex'));
    assert.equal(asset.units, 'm');
    assert.equal(asset.tolerance_mm, 3);
    assert.equal(asset.parts.length, 425);
    assert.equal(new Set(asset.parts.map(part => part.name)).size, 425);
    assert.ok(!asset.parts.some(part => part.name === 'AirframeEnvelope'));
    for (const part of asset.parts) {
      assert.equal(part.vertices.length % 3, 0, part.name);
      assert.equal(part.indices.length % 3, 0, part.name);
      assert.ok(part.vertices.length > 0 && part.indices.length > 0, part.name);
      assert.ok(part.vertices.every(Number.isFinite), part.name);
      assert.ok(part.indices.every(index => Number.isInteger(index) && index >= 0 && index < part.vertices.length / 3), part.name);
    }
  });
}

test('C2 mesh manifest matches generated files and 12 CAD frames are present', async () => {
  const manifest = await readJson('../assets/c2/manifest.json');
  for (const state of states) {
    const meshPath = new URL(`../assets/c2/${state}.json`, import.meta.url);
    const file = await readFile(meshPath);
    assert.equal(manifest[state].parts, 425);
    assert.equal(manifest[state].bytes, file.byteLength);
    assert.equal(manifest[state].sha256, (await readJson(`../assets/c2/${state}.json`)).source_sha256);
  }
  for (let frame = 0; frame < 12; frame++) {
    const file = await stat(new URL(`../assets/c2/motion-${String(frame).padStart(2, '0')}.png`, import.meta.url));
    assert.ok(file.size > 0);
  }
});

test('C2 HTML local references and required handles resolve', async () => {
  const html = await readFile(new URL('../c2.html', import.meta.url), 'utf8');
  const ids = new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]));
  assert.match(html, /href="\.\/index\.html"/);
  assert.match(html, /href="\.\/c1\.html"/);
  assert.match(html, /id="motionMode" class="active"/);
  assert.match(html, /id="viewport" tabindex/);
  assert.doesNotMatch(html, /id="motionViewport"/);
  for(const id of ['collective','cyclic','phase','azimuth','motionPlay','residual'])assert.ok(ids.has(id));
  assert.match(html, /results\/C2/);
  assert.match(html, /425/);
  assert.match(html, /648/);
  assert.match(html, /G1[\s\S]*未通過/);
  assert.match(html, /R-11[\s\S]*R-15[\s\S]*R-16/);
  assert.match(html, /コストについて/);
  for (const [, path] of html.matchAll(/(?:src|href)="([^" ]+)"/g)) {
    if (path.startsWith('#')) {
      assert.ok(path === '#' || ids.has(path.slice(1)), path);
      continue;
    }
    if (/^(https:|data:)/.test(path)) continue;
    assert.ok((await readFile(new URL(`../${path}`, import.meta.url))).length > 0, path);
  }
});

test('C2 copied verification files contain no local absolute paths', async () => {
  const files = ['../../results/C2/verification/cad-checks.json', '../../results/C2/final-report.md', '../../results/C2/RUN.md'];
  for (const file of files) {
    const text = await readFile(new URL(file, import.meta.url), 'utf8');
    assert.doesNotMatch(text, /C:\\\\Users\\\\|Dropbox|LOCALAPPDATA|\/home\//i, file);
  }
});
