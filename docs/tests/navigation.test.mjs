import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

test('landing page and both design pages have working local links',async()=>{
  for(const file of ['index.html','c1.html','c2.html']){
    const html=await readFile(new URL(`../${file}`,import.meta.url),'utf8');
    for(const [,href] of html.matchAll(/(?:href|src)="([^" ]+)"/g)){
      if(/^(https?:|data:|#)/.test(href))continue;
      const [path,anchor]=href.split('#');
      const content=await readFile(new URL(`../${path}`,import.meta.url));
      assert.ok(content.length,`${file}: ${path}`);
      if(anchor)assert.ok(content.toString().includes(`id="${anchor}"`),`${path}#${anchor}`);
    }
  }
});

test('old C1 hash URLs redirect while the new comparison stays on the landing',async()=>{
  const script=await readFile(new URL('../landing.mjs',import.meta.url),'utf8');
  for(const base of ['https://naoyamd.github.io/rotor-cad-benchmark/','https://rotor.nightly.dedyn.io/'])
  for(const hash of ['','#comparison','#explore','#benchmark','#evidence']){
    let redirected;
    const location={hash,href:`${base}${hash}`,replace:url=>{redirected=url.href;}};
    let onHashChange;
    vm.runInNewContext(script,{location,URL,addEventListener:(event,fn)=>{assert.equal(event,'hashchange');onHashChange=fn;}});
    assert.equal(redirected,['#explore','#benchmark','#evidence'].includes(hash)?`${base}c1.html${hash}`:undefined);
    location.hash='#explore';
    onHashChange();
    assert.equal(redirected,`${base}c1.html#explore`);
  }
});
