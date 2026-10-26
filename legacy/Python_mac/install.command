#!/bin/bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$TASK_DIR"

if [ "$(uname -s)" != "Darwin" ]; then
    echo "This installer targets macOS / Apple Silicon. See README.md."
    exit 1
fi
if [ "$(id -u)" = "0" ]; then
    echo "Run without sudo. Only the application's IP alias setup needs sudo."
    exit 1
fi
PYTHON_BIN="${MOTOR_PYTHON:-}"
if [ -z "$PYTHON_BIN" ]; then
    if [ -x ".venv/bin/python" ]; then
        PYTHON_BIN="$TASK_DIR/.venv/bin/python"
    else
        for MOTOR_CANDIDATE in python3 python3.12 python3.13 /Library/Frameworks/Python.framework/Versions/3.12/bin/python3.12 /Library/Frameworks/Python.framework/Versions/3.13/bin/python3.13; do
            if command -v "$MOTOR_CANDIDATE" >/dev/null 2>&1 && "$MOTOR_CANDIDATE" -c 'import sys; sys.exit(0 if sys.version_info[:2] in ((3,12),(3,13)) else 1)' >/dev/null 2>&1; then
                PYTHON_BIN="$MOTOR_CANDIDATE"
                break
            fi
        done
    fi
fi
if [ -z "$PYTHON_BIN" ] || ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    echo "Python 3.12 or 3.13 required. Install the macOS universal2 package from python.org."
    echo "See README.md, including Install Certificates.command."
    exit 1
fi
MOTOR_CHECK_PYTHON='import platform,sys,sysconfig; assert sys.version_info[:2] in ((3,12),(3,13)), "Python 3.12 or 3.13 required"; assert platform.machine() == "arm64", "Use a native arm64 Terminal / Python on M3 (disable Rosetta)"; assert not sysconfig.get_config_var("Py_GIL_DISABLED"), "Use standard CPython, not the free-threaded build"; import tkinter; print("Using", sys.version.split()[0], platform.machine())'
"$PYTHON_BIN" -c "$MOTOR_CHECK_PYTHON"
if [ ! -x ".venv/bin/python" ]; then
    "$PYTHON_BIN" -m venv .venv
fi
.venv/bin/python -c "$MOTOR_CHECK_PYTHON"
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -c 'import tkinter,matplotlib,prompt_toolkit; import hashlib,pathlib; pathlib.Path(".venv/requirements.sha256").write_text(hashlib.sha256(pathlib.Path("requirements.txt").read_bytes()).hexdigest())'
echo "Installation complete. First run: bash run.command --dry-run"
