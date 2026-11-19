"""Read-only identity check for the validated ForestScene short case; never starts flight."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

DEMO = Path(__file__).resolve().parent
REPO = DEMO.parents[3]


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def check(repo, release, settings, lock, local=None):
    errors = []
    def matches(path, expected, label):
        if not path.is_file() or sha(path) != expected:
            errors.append(label)
    matches(release / 'manifest.json', lock['release_manifest_sha256'], 'release manifest')
    if errors:
        return errors
    manifest = json.loads((release / 'manifest.json').read_text())
    if manifest['status'] != 'built' or manifest['images'] != lock['images']:
        errors.append('build status/images')
    for rel, value in lock['source_files'].items():
        matches(repo / rel, value, 'source: ' + rel)
    for rel, value in manifest['artifacts'].items():
        matches(release / rel, value, 'artifact: ' + rel)
    for rel, value in lock['extra_release_files'].items():
        matches(release / rel, value, 'release support: ' + rel)
    matches(settings, lock['settings_sha256'], 'settings (does not verify the loaded UE map)')
    if local is not None:
        if local.get('diagnostics') != lock['diagnostics']:
            errors.append('diagnostics differs from validated case')
        if Path(local['run_root']).exists():
            errors.append('run_root already exists; preserve previous evidence')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local', type=Path, required=True)
    parser.add_argument('--images', action='store_true', help='also inspect the four local Docker images; no pull or container startup')
    args = parser.parse_args()
    local = json.loads(args.local.read_text(encoding='utf-8'))
    lock = json.loads((DEMO / 'forest-short.lock.json').read_text())
    errors = check(REPO, Path(local['release']), Path(local['settings']), lock, local)
    if args.images:
        for role, expected in lock['images'].items():
            found = subprocess.run(['docker', 'image', 'inspect', expected, '--format', '{{.Id}}'],
                                   capture_output=True, text=True, timeout=30)
            if found.returncode or found.stdout.strip() != expected:
                errors.append('image unavailable: ' + role)
    print(json.dumps({'pass': not errors, 'baseline': lock['id'], 'errors': errors,
                      'scope': 'file/image identity only; verify actual ForestScene, fresh session and resource availability before flight'}, indent=2))
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
