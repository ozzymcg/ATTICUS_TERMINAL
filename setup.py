from pathlib import Path
from setuptools import setup, find_packages

ROOT = Path(__file__).resolve().parent
SKIP_DIRS = {".git", "__pycache__", "build", "dist", ".venv", "venv"}
SKIP_SUFFIXES = {".py", ".pyc", ".pyo"}

def collect_data_files():
    """Collect non-Python project files for source/wheel installs."""
    items = []
    for path in ROOT.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(ROOT)
        if any(part in SKIP_DIRS for part in rel.parts):
            continue
        if path.suffix.lower() in SKIP_SUFFIXES:
            continue
        if rel.parts and rel.parts[0] in {"build", "dist"}:
            continue
        dest = str(Path("atticus_terminal_data") / rel.parent).replace("\\", "/")
        items.append((dest, [str(path)]))
    return items

setup(
    name="atticus-terminal",
    version="2.0.0",
    description="Atticus terminal path planner and simulator",
    packages=find_packages(include=["mod", "mod.*"]),
    py_modules=["main"],
    include_package_data=True,
    package_data={"": ["*.json"]},
    data_files=collect_data_files(),
    entry_points={
        "console_scripts": [
            "atticus-terminal=main:main",
        ]
    },
    install_requires=[
        "pygame>=2.1.0",
    ],
    python_requires=">=3.8",
)
