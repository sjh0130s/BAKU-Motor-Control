@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    call install.cmd
    if errorlevel 1 exit /b 1
)
.venv\Scripts\python.exe -c "import hashlib,pathlib,sys; p=pathlib.Path('.venv/requirements.sha256'); sys.exit(0 if p.exists() and p.read_text()==hashlib.sha256(pathlib.Path('requirements.txt').read_bytes()).hexdigest() else 1)"
if errorlevel 1 (
    call install.cmd
    if errorlevel 1 exit /b 1
)
.venv\Scripts\python.exe motor_test.py %*
exit /b %errorlevel%
