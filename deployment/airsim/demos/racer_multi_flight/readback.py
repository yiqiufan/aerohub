"""Read finalized flight diagnostic bags in an offline fixed-image container."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import uuid

DEMO = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new directory; never overwrite previous readback')
    args = parser.parse_args()
    root, release, out = (p.resolve() for p in (args.run_root, args.release, args.output))
    def digest(p):
        with p.open('rb') as f:
            return hashlib.file_digest(f, 'sha256').hexdigest()
    if digest(root / 'build-manifest.json') != digest(release / 'manifest.json'):
        raise ValueError('readback release differs from recorded build')
    build = json.loads((release / 'manifest.json').read_text())
    for rel, value in build['artifacts'].items():
        if digest(release / rel) != value:
            raise ValueError('artifact mismatch: ' + rel)
    # A finished flow and finalized recorder results are required. A failed flight
    # may still have valid diagnostic data; do not turn its verdict into success.
    json.loads((root / 'flow-result.json').read_text())
    fleet = json.loads((root / 'configuration/fleet.json').read_text())
    if len(fleet['vehicles']) != 2:
        raise ValueError('this readback is scoped to the two-UAV case')
    for v in fleet['vehicles']:
        json.loads((root / 'diagnostics' / v['namespace'] / 'recorder-result.json').read_text())
    out.mkdir(parents=True, exist_ok=False)
    name = 'racer-readback-' + uuid.uuid4().hex[:12]
    cmd = ['docker', 'run', '--rm', '--name', name, '--network', 'none', '--pull=never',
           '-e', 'PYTHONDONTWRITEBYTECODE=1']
    for source, target, readonly in [(root, '/evidence', True), (out, '/result', False),
                                   (release, '/release', True), (DEMO, '/code', True),
                                   (DEMO.parent / 'racer_multi_ground', '/ground-code', True)]:
        cmd += ['--mount', f'type=bind,source={source.as_posix()},target={target}' + (',readonly' if readonly else '')]
    setup = ''.join('source ' + p + '/setup.bash\n' for p in
                    ['/opt/ros/humble', '/root/aerostack2_ws/install', '/opt/px4_ws/install', '/opt/uav_execution_ws/install'])
    cmd += ['--entrypoint', 'bash', build['images']['onboard'], '-lc', setup +
            'source /release/native/install/local_setup.bash\nexec python3 /code/readback_flight.py']
    (out / 'command.json').write_text(json.dumps(cmd, indent=2))
    try:
        with (out / 'console.log').open('x', encoding='utf-8') as stream:
            code = subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT, timeout=300).returncode
    except subprocess.TimeoutExpired:
        subprocess.run(['docker', 'stop', '-t', '10', name], capture_output=True, timeout=30)
        (out / 'timeout.json').write_text(json.dumps({'pass': False, 'container': name, 'timeout_s': 300}))
        raise
    print(json.dumps({'exit_code': code, 'report': str(out / 'recording-result.json')}))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
