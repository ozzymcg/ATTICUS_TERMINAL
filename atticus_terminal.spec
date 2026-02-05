# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec for Atticus Terminal.
Bundles full project data for one-click, cross-machine portability.
"""
import os, sys
from PyInstaller.utils.hooks import collect_submodules

block_cipher = None

# Some PyInstaller entry modes don't populate __file__; fall back to argv[0].
spec_path = os.path.abspath(sys.argv[0])
project_root = os.path.abspath(os.path.dirname(spec_path))

_datas = []
_skip_dirs = {
    ".git", "__pycache__", "build", "dist", ".venv", "venv",
    ".mypy_cache", ".pytest_cache",
}
_skip_ext = {".py", ".pyc", ".pyo"}

def _add_data_file(path):
    rel = os.path.relpath(path, project_root).replace("\\", "/")
    dest = os.path.dirname(rel) or "."
    _datas.append((path, dest))

for root, dirs, files in os.walk(project_root):
    rel_root = os.path.relpath(root, project_root).replace("\\", "/")
    dirs[:] = [d for d in dirs if d not in _skip_dirs]
    if rel_root.startswith("dist") or rel_root.startswith("build"):
        continue
    for name in files:
        ext = os.path.splitext(name)[1].lower()
        if ext in _skip_ext:
            continue
        src = os.path.join(root, name)
        _add_data_file(src)

# collect all modules inside mod/* so PyInstaller doesn't miss anything
_hidden = collect_submodules("mod")

a = Analysis(
    ['main.py'],
    pathex=[project_root],
    binaries=[],
    datas=_datas,
    hiddenimports=_hidden,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='ATTICUS_TERMINAL',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    icon=os.path.join(project_root, "ATTICUS.png"),
)
