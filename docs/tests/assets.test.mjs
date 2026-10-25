import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';

for (const state of ['neutral','max-input','parked','folded']) {
  test(`${state}: all 145 CAD parts trace to the original and have valid meshes`, async () => {
    const asset=JSON.parse(await readFile(new URL(`../assets/${state}.json`,import.meta.url)));
    const cad=await readFile(new URL(`../../results/C1/rotor-${state}.FCStd`,import.meta.url));
    assert.equal(asset.source_sha256,createHash('sha256').update(cad).digest('hex'));
    assert.equal(asset.parts.length,145);
    assert.equal(new Set(asset.parts.map(p=>p.name)).size,145);
    assert.equal(asset.units,'m');
    for(const p of asset.parts){
      assert.equal(p.vertices.length%3,0);assert.equal(p.indices.length%3,0);
      assert.ok(p.vertices.length>0 && p.indices.length>0);
      assert.ok(p.vertices.every(Number.isFinite));
      assert.ok(p.indices.every(i=>Number.isInteger(i)&&i>=0&&i<p.vertices.length/3));
    }
  });
}

test('local HTML assets and internal anchors resolve',async()=>{
  const html=await readFile(new URL('../index.html',import.meta.url),'utf8');
  const ids=new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]));
  for(const [,path] of html.matchAll(/(?:src|href)="([^" ]+)"/g)){
    if(path.startsWith('#')){assert.ok(path==='#'||ids.has(path.slice(1)),path);continue;}
    if(/^(https:|data:)/.test(path))continue;
    assert.ok((await readFile(new URL(`../${path}`,import.meta.url))).length>0,path);
  }
});
