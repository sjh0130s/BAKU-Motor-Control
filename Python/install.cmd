@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    py -3.13 -m venv .venv
    if errorlevel 1 goto fail
)
.venv\Scripts\python.exe -c "import sys; assert sys.version_info[:2] == (3, 13), 'Recreate .venv with Python 3.13'"
if errorlevel 1 goto fail
.venv\Scripts\python.exe -m pip install -r requirements.txt
if errorlevel 1 goto fail
.venv\Scripts\python.exe -c "import tkinter, matplotlib, prompt_toolkit; import hashlib,pathlib; pathlib.Path('.venv/requirements.sha256').write_text(hashlib.sha256(pathlib.Path('requirements.txt').read_bytes()).hexdigest())"
if errorlevel 1 goto fail
echo Installation complete. Run run.cmd --dry-run first.
exit /b 0
:fail
echo Installation failed. Install Python 3.13 64-bit with pip, Tcl/Tk, and Python launcher. See README.md.
exit /b 1
