"""Run local environment checks for Atticus Terminal."""

import os
import platform
import sys
from pathlib import Path


def _ok(flag: bool) -> str:
    return "PASS" if flag else "FAIL"


def _warn(flag: bool) -> str:
    return "PASS" if flag else "WARN"


def _can_write(path: Path) -> bool:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        probe = path.parent / ".atticus_write_test"
        probe.write_text("ok", encoding="utf-8")
        probe.unlink(missing_ok=True)
        return True
    except Exception:
        return False


def main() -> int:
    root = Path(__file__).resolve().parent
    print("Atticus Terminal Doctor")
    print(f"- OS: {platform.system()} {platform.release()}")
    print(f"- Python: {platform.python_version()} ({sys.executable})")

    py_ok = sys.version_info >= (3, 8)
    print(f"[{_ok(py_ok)}] Python >= 3.8")
    if not py_ok:
        return 1

    win_store = ("WindowsApps" in sys.executable) or ("PythonSoftwareFoundation" in sys.executable)
    print(f"[{_warn(True)}] Windows Store Python detected: {win_store}")
    if win_store:
        print("       Tip: use `py -m pip install -r requirements.txt`.")

    try:
        import tkinter  # noqa: F401
        tk_ok = True
    except Exception:
        tk_ok = False
    print(f"[{_ok(tk_ok)}] tkinter available")

    try:
        import pygame  # noqa: F401
        pg_ok = True
    except Exception:
        pg_ok = False
    print(f"[{_ok(pg_ok)}] pygame available")

    required = [
        root / "main.py",
        root / "config.json",
        root / "ATTICUS.png",
        root / "mod" / "config.py",
        root / "mod" / "mcl_codegen.py",
    ]
    files_ok = all(p.exists() for p in required)
    print(f"[{_ok(files_ok)}] core files present")
    if not files_ok:
        for p in required:
            if not p.exists():
                print(f"       Missing: {p}")
    docs_dir = root / "pros" / "docs"
    docs_found = docs_dir.exists() and any(docs_dir.glob("*.md"))
    print(f"[{_warn(docs_found)}] pros docs detected for MCL export")

    try:
        from mod.config import _config_candidates  # type: ignore

        cfg_candidates = [Path(p) for p in _config_candidates()]
    except Exception:
        cfg_candidates = [root / "config.json"]
    writable = any(_can_write(p) for p in cfg_candidates)
    print(f"[{_ok(writable)}] writable config path available")

    all_ok = py_ok and tk_ok and pg_ok and files_ok and writable
    if all_ok:
        print("All checks passed.")
        return 0
    print("One or more checks failed.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
