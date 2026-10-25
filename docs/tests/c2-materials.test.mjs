import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {surfaceFor} from '../c2-materials.mjs';
test('C2 display finishes respect BOM material before part-name hints',async()=>{
  const {parts}=JSON.parse(await readFile(new URL('../assets/c2/neutral.json',import.meta.url)));
  const finish=name=>surfaceFor(parts.find(p=>p.name===name));
  assert.equal(finish('Blade1Elastomer').metalness,0);
  assert.ok(finish('Blade1Airfoil').metalness<.1);
  assert.equal(finish('BearingCage').metalness,0);
  assert.notEqual(finish('GuideBush100').color,finish('Mast').color);
  assert.ok(finish('Servo0Rod').roughness<finish('Servo0Body').roughness);
  for(const part of parts){const s=surfaceFor(part);assert.ok(Number.isInteger(s.color));assert.ok(s.metalness>=0&&s.metalness<=1&&s.roughness>=0&&s.roughness<=1);}
});
