#!/bin/bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$TASK_DIR"

if [ "$(uname -s)" != "Darwin" ]; then
    echo "This launcher targets macOS. For portable simulation use python3 motor_test.py --dry-run."
    exit 1
fi
if [ "$(id -u)" = "0" ]; then
    echo "Run without sudo. IP alias changes request sudo when needed."
    exit 1
fi
if [ ! -x ".venv/bin/python" ]; then
    bash "$TASK_DIR/install.command"
fi
if ! .venv/bin/python -c 'import hashlib,pathlib,sys; p=pathlib.Path(".venv/requirements.sha256"); sys.exit(0 if p.exists() and p.read_text()==hashlib.sha256(pathlib.Path("requirements.txt").read_bytes()).hexdigest() else 1)'; then
    bash "$TASK_DIR/install.command"
fi
exec "$TASK_DIR/.venv/bin/python" "$TASK_DIR/motor_test.py" "$@"
