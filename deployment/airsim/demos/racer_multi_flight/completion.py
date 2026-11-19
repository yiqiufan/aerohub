"""Task-level completion checks, separate from RACER planning and physical recovery."""
import math

def validate_budget(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
        raise ValueError('explore-complete requires a finite positive completion_wall_budget_s')
    return float(value)

def natural_finish(state, session, generation):
    return bool(session and state.get('native_session')==session
        and state.get('trigger_generation')==generation+1
        and state.get('fsm')=='FINISH'
        and state.get('completion_reason')=='native_return_finished'
        and not state.get('process_failure') and not state.get('completion_fault'))

def fleet_finished(rows, members, now, freshness):
    return set(rows)==set(members) and all(
        r.get('natural_finished') and not r.get('failed') and 0<=now-r['at']<=freshness
        for r in rows.values())
