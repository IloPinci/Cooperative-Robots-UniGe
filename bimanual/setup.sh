#!/usr/bin/env bash
set -euo pipefail
exercise_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -E -m venv "$exercise_dir/.venv"
"$exercise_dir/.venv/bin/python" -E -m pip install --no-cache-dir --only-binary=:all: -r "$exercise_dir/requirements-linux.txt"
"$exercise_dir/.venv/bin/python" -E -m pip check
"$exercise_dir/.venv/bin/python" -E "$exercise_dir/check_setup.py"
