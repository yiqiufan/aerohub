"""Run the bounded two-UAV case from a freshly reset UE session; retain every stage."""
import argparse
import importlib.util
import json
import time
from pathlib import Path


def execute(deployment, mode="explore"):
    d = deployment
    if mode=="explore-complete":d.validate_completion_config()
    stages = []
    def stage(name, operation):
        entry = {'stage': name, 'started_ns': time.time_ns()}
        stages.append(entry)
        try:
            code = operation()
            entry['returncode'] = code or 0
            if code:
                raise RuntimeError(f'{name} returned {code}')
        except Exception as exc:
            entry['error'] = str(exc)
            raise
        finally:
            entry['ended_ns'] = time.time_ns()
            if d.root.exists():
                (d.root / 'flow-stages.json').write_text(json.dumps(stages, indent=2))
    try:
        stage('up', d.up)
        # Core startup is asynchronous. Wait for real ground evidence, never a fixed delay.
        def ready():
            deadline = time.monotonic() + 60
            while True:
                try:
                    d.ground(d.root)
                    return
                except Exception:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(1)
        stage('ground-ready', ready)
        stage('sensors', d.sensors)
        stage('survey', d.survey)
        stage('start', d.start)
        stage('logger-start', d.logger)
        task_error = None
        try:
            stage(mode, lambda: d.mission_mode(mode))
        except Exception as exc:
            task_error = str(exc)
        # Failed mission may have recovered. Ground confirmation still gates every cleanup.
        stage('ground-after-task', lambda: d.ground(d.root))
        stage('logger-stop', lambda: d.logger(True))
        stage('stop', d.stop)
        stage('down', d.down)
        if task_error:
            raise RuntimeError(task_error)
        result = json.loads((d.root / 'mission/result.json').read_text())
        if not result.get('pass'):
            raise RuntimeError('mission result missing successful verdict')
        outcome = {'pass': True, 'stages': stages}
    except Exception as exc:
        # Do not tear down flight services when landing, startup or ownership is uncertain.
        outcome = {'pass': False, 'error': str(exc), 'stages': stages,
                   'recovery': 'Inspect stages and actual PX4 state; retain flight services until ground confirmed.'}
    if d.root.exists():
        (d.root / 'flow-result.json').write_text(json.dumps(outcome, indent=2))
    print(json.dumps(outcome, indent=2), flush=True)
    return 0 if outcome['pass'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local', required=True, type=Path)
    parser.add_argument('--mode', choices=['explore','explore-complete','takeoff-drone1'], default='explore')
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('flight_entry', Path(__file__).with_name('run.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    deployment = module.FlightDeployment(args.local)
    if deployment.root.exists():
        raise SystemExit('Use a new run_root; existing evidence must be preserved')
    raise SystemExit(execute(deployment, args.mode))
