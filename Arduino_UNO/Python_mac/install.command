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
PYTHON_BIN="${MOTOR_PYTHON:-python3.13}"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    PYTHON_BIN="/Library/Frameworks/Python.framework/Versions/3.13/bin/python3.13"
fi
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    echo "Install the Python 3.13 macOS universal2 package from python.org first."
    echo "See README.md, including Install Certificates.command."
    exit 1
fi
"$PYTHON_BIN" -c 'import platform,sys,sysconfig; assert sys.version_info[:2] == (3,13), "Python 3.13 required"; assert platform.machine() == "arm64", "Use a native arm64 Terminal / Python on M3 (disable Rosetta)"; assert not sysconfig.get_config_var("Py_GIL_DISABLED"), "Use standard CPython, not the free-threaded build"; import tkinter'
if [ ! -x ".venv/bin/python" ]; then
    "$PYTHON_BIN" -m venv .venv
fi
.venv/bin/python -c 'import platform,sys; assert sys.version_info[:2] == (3,13), "Recreate .venv with Python 3.13"; assert platform.machine() == "arm64", "Recreate .venv with arm64 Python"'
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -c 'import tkinter,matplotlib,prompt_toolkit; import hashlib,pathlib; pathlib.Path(".venv/requirements.sha256").write_text(hashlib.sha256(pathlib.Path("requirements.txt").read_bytes()).hexdigest())'
echo "Installation complete. First run: bash run.command --dry-run"
