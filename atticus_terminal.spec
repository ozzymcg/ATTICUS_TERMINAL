# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec for Atticus Terminal.
Bundles config JSONs and icon so a one-click build works.
"""
import os, sys
from PyInstaller.utils.hooks import collect_submodules

block_cipher = None

# Some PyInstaller entry modes don't populate __file__; fall back to argv[0].
spec_path = os.path.abspath(sys.argv[0])
project_root = os.path.abspath(os.path.dirname(spec_path))

# data files to ship alongside the exe
_datas = [
    (os.path.join(project_root, "ATTICUS.png"), "."),
    (os.path.join(project_root, "config.json"), "."),
    (os.path.join(project_root, "demo_autonskills.json"), "."),
    (os.path.join(project_root, "mod", "config.json"), "mod"),
]

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
