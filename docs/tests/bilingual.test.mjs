import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {bi} from '../bilingual.mjs';

test('dynamic bilingual labels keep English first and escape source text',()=>{
  const markup=bi('<script>&"', '説明 <img>');
  assert.match(markup,/^<span class="en" lang="en">&lt;script&gt;&amp;&quot;/);
  assert.match(markup,/<span class="ja" lang="ja">説明 &lt;img&gt;/);
  assert.ok(!markup.includes('<script>')&&!markup.includes('<img>'));
});
test('all public pages have English-first markup and unique control IDs',async()=>{
  for(const file of ['index.html','c1.html','c2.html']){
    const html=await readFile(new URL(`../${file}`,import.meta.url),'utf8');
    assert.match(html,/<html lang="en">/);
    assert.match(html,/bilingual\.css/);
    assert.ok((html.match(/class="en"/g)||[]).length>25);
    assert.ok((html.match(/class="ja"/g)||[]).length>25);
    const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
    assert.equal(new Set(ids).size,ids.length,file);
  }
});
test('English OGP is C2-only and its PNG is 1200 by 630',async()=>{
  const c2=await readFile(new URL('../c2.html',import.meta.url),'utf8');
  assert.match(c2,/property="og:locale" content="en_US"/);
  assert.match(c2,/name="twitter:card" content="summary_large_image"/);
  assert.match(c2,/property="og:image" content="https:\/\/rotor\.nightly\.dedyn\.io\/assets\/c2-ogp\.png"/);
  for(const file of ['index.html','c1.html'])assert.ok(!(await readFile(new URL(`../${file}`,import.meta.url),'utf8')).includes('property="og:'));
  const png=await readFile(new URL('../assets/c2-ogp.png',import.meta.url));
  assert.equal(png.readUInt32BE(16),1200);assert.equal(png.readUInt32BE(20),630);
});
