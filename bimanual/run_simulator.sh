#!/usr/bin/env bash
set -euo pipefail
exercise_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$exercise_dir/.venv/bin/python" ]]; then
    echo "Run $exercise_dir/setup.sh first." >&2
    exit 1
fi
cd "$exercise_dir/simulator"
exec "$exercise_dir/.venv/bin/python" -E -u franka_panda_simulation.py
