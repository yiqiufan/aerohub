#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_DIR}"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if [ "$#" -eq 0 ]; then
  set -- --runs 5 --timeout-sec 120 --output benchmark_results/pillar_runs.csv
fi

python3 tools/benchmark_pillar.py "$@"
