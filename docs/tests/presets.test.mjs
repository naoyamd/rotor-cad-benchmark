import {test} from 'node:test';
import assert from 'node:assert/strict';
import {pitchPresets, speedPresets} from '../presets.mjs';
import {solve} from '../kinematics.mjs';
test('three distinct pitch presets remain inside the finite-link envelope',()=>{
  assert.equal(pitchPresets.length,3);
  const pitches=pitchPresets.map(p=>solve(0,p.collective,p.cyclic,0).theta);
  assert.ok(pitches[1]-pitches[0]>4 && pitches[2]-pitches[1]>4);
  for(const p of pitchPresets)for(let a=0;a<360;a+=6)assert.ok(solve(a,p.collective,p.cyclic,0).residual<1e-9);
});
test('three visual speeds are ordered and slower than the design speed',()=>{
  assert.equal(speedPresets.length,3);
  assert.ok(speedPresets.every((rpm,i)=>rpm>0&&rpm<210&&(i===0||rpm>speedPresets[i-1])));
});
