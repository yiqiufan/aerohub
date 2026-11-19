"""Read-only FIXED-CASE identity check; not a new-environment readiness check. Never flies."""
import argparse
import json
from pathlib import Path
import subprocess

from check_baseline import check, sha

DEMO = Path(__file__).resolve().parent
REPO = DEMO.parents[3]


def selected_errors(local_path, local, lock):
    errors = []
    budget = local.get('completion_wall_budget_s')
    if isinstance(budget, bool) or budget != lock['completion_wall_budget_s']:
        errors.append('completion budget differs from validated case')
    for key, expected in lock['selected_configs'].items():
        value = local.get(key)
        if not isinstance(value, str) or not value:
            errors.append('explicit configuration required: ' + key)
            continue
        path = Path(value)
        if not path.is_absolute():
            path = local_path.parent / path
        if not path.is_file() or sha(path) != expected:
            errors.append('selected configuration differs: ' + key)
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog=(
        'Environment-specific paths/addresses belong in local JSON. Changed settings, fleet/flight '
        'parameters or budget require separate validation; a mismatch is not proof of incompatibility. '
        'Do not rewrite the baseline to approve a new environment. See FOREST_COMPLETE.md.'))
    parser.add_argument('--local', type=Path, required=True)
    parser.add_argument('--images', action='store_true', help='inspect local images; never pull or start containers')
    parser.add_argument('--pc-dual', action='store_true', help='check the PC dual release with the R1 recorder')
    args = parser.parse_args()
    try:
        local = json.loads(args.local.read_text(encoding='utf-8'))
        lock = json.loads((DEMO / 'forest-complete.lock.json').read_text(encoding='utf-8'))
        if args.pc_dual:
            update = json.loads((DEMO / 'pc-dual.lock.json').read_text(encoding='utf-8'))
            if sha(DEMO / 'forest-complete.lock.json') != update['base_sha256']:
                raise ValueError('PC dual base lock differs')
            lock['id'] = update['id']
            lock['source_files'].update(update['source_files'])
            lock['delivery_files'].update(update['delivery_files'])
        errors = selected_errors(args.local.resolve(), local, lock)
        errors += check(REPO, Path(local['release']), Path(local['settings']), lock, local)
        for rel, expected in lock['delivery_files'].items():
            path = REPO / rel
            if not path.is_file() or sha(path) != expected:
                errors.append('delivery file: ' + rel)
        if args.images:
            for role, expected in lock['images'].items():
                found = subprocess.run(['docker', 'image', 'inspect', expected, '--format', '{{.Id}}'],
                                       capture_output=True, text=True, timeout=30)
                if found.returncode or found.stdout.strip() != expected:
                    errors.append('image unavailable: ' + role)
    except (OSError, KeyError, ValueError, TypeError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({'pass': False, 'error': str(exc)}))
        return 1
    print(json.dumps({'pass': not errors, 'baseline': lock['id'], 'errors': errors,
                      'check_kind': 'fixed-case-identity', 'deployment_readiness': 'not_assessed',
                      'scope': 'file/config/image identity only; not scene readiness or flight acceptance'}, indent=2))
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
