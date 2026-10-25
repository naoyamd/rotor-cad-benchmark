import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

test('displayed benchmark costs, tokens and times match the published source',async()=>{
  const html=await readFile(new URL('../c1.html',import.meta.url),'utf8');
  const summary=JSON.parse(await readFile(new URL('../../metrics/C1/summary.json',import.meta.url)));
  const rows=[...html.matchAll(/<tr data-benchmark="([^"]+)">([\s\S]*?)<\/tr>/g)];
  assert.equal(rows.length,3);
  for(const [,key,row] of rows){
    const cells=[...row.matchAll(/data-metric="([^"]+)" data-value="([^"]+)">([^<]+)/g)];
    assert.equal(cells.length,4);
    for(const [,metric,value,visible] of cells){
      assert.equal(Number(value),Number(summary[key][metric]));
      if(metric.endsWith('_usd'))assert.equal(visible,`$${Number(value).toFixed(2)}`);
      else if(metric==='total_tokens')assert.equal(visible,Number(value).toLocaleString('en-US'));
      else{const seconds=Math.round(Number(value)/1000),h=Math.floor(seconds/3600),m=Math.floor(seconds%3600/60),s=String(seconds%60).padStart(2,'0');assert.equal(visible,`${h?h+'時間':''}${m}分${s}秒`);}
    }
  }
  const requests=JSON.parse(await readFile(new URL('../../metrics/C1/requests.json',import.meta.url)));
  assert.equal(requests.filter(r=>r.model==='gpt-6-astra'&&r.reasoning_effort==='medium').length,139);
  assert.equal(requests.filter(r=>r.model==='gpt-5.6-luna'&&r.reasoning_effort==='max').length,110);
});
