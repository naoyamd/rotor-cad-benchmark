"""Recompute disclosed execution metrics and API estimate from sanitized records."""
import json
from collections import defaultdict
from decimal import Decimal
from pathlib import Path

HERE=Path(__file__).resolve().parent
FIELDS=['input_tokens','cached_input_tokens','cache_write_input_tokens','output_tokens','reasoning_output_tokens','total_tokens']

def summarize():
    requests=json.loads((HERE/'requests.json').read_text(encoding='utf-8'))
    timing=json.loads((HERE/'timing.json').read_text(encoding='utf-8'))
    rates=json.loads((HERE/'rates.json').read_text(encoding='utf-8'))
    buckets={}
    for key,phases in [('specification',['specification','requirements_review']),('cad_design',['cad_design']),('total',['specification','requirements_review','cad_design'])]:
        records=[r for r in requests if r['phase'] in phases]
        usage={k:sum(r[k] for r in records) for k in FIELDS}
        models={}
        for model in sorted({r['model'] for r in records}):
            rr=[r for r in records if r['model']==model]
            u={k:sum(r[k] for r in rr) for k in FIELDS}
            price=rates['per_million_tokens'][model]
            assert all(r['input_tokens']<=rates['short_context_max_input_tokens'] for r in rr)
            assert u['cache_write_input_tokens']==0
            terms=[(u['input_tokens']-u['cached_input_tokens'])*Decimal(str(price['uncached_input'])),
                   u['cached_input_tokens']*Decimal(str(price['cached_input'])),u['output_tokens']*Decimal(str(price['output']))]
            cost=sum(terms)/Decimal(1000000)
            models[model]={'requests':len(rr),**u,'uncached_input_tokens':u['input_tokens']-u['cached_input_tokens'],
                           'standard_usd':str(cost),'cost_components_usd':[str(t/Decimal(1000000)) for t in terms]}
        gross=sum(t['duration_ms'] for t in timing['turns'] if t['primary'] and t['phase'] in phases)
        # These two verified waits do not overlap each other or any subagent turn.
        excluded=sum(t['excluded_ms'] for t in timing['excluded_waits'] if t['phase'] in phases)
        cost=sum(Decimal(m['standard_usd']) for m in models.values())
        buckets[key]={'primary_elapsed_ms':gross,'excluded_wait_ms':excluded,'adjusted_elapsed_ms':gross-excluded,
                      'subagent_turn_duration_sum_ms':sum(t['duration_ms'] for t in timing['turns'] if not t['primary'] and t['phase'] in phases),
                      'requests':len(records),**usage,'uncached_input_tokens':usage['input_tokens']-usage['cached_input_tokens'],
                      'standard_token_only_usd':str(cost),'fast_token_only_usd':str(cost*2),'models':models}
    total=buckets['total']
    assert total['total_tokens']==total['input_tokens']+total['output_tokens']
    for field in ['adjusted_elapsed_ms','total_tokens','requests']:
        assert total[field]==buckets['specification'][field]+buckets['cad_design'][field]
    assert len({r['request'] for r in requests})==len(requests)
    return buckets

if __name__=='__main__':
    result=summarize()
    (HERE/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
