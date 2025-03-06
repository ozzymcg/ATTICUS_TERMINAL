from cx_Freeze import setup, Executable

# Define build options
build_exe_options = {
    "packages": ["os", "sys", "pygame", "tkinter"],
    # to rename or move them, use tuples: (source, destination)
    "include_files": [
        "ATTICUS.png",
        "prototype_auton.json"
    ],
}

setup(
    name="ATTICUS_TERMINAL",
    version="1.5.1",
    description="VEX Field and Autonomous Simulation.",
    options={"build_exe": build_exe_options},
    executables=[Executable("SKELETON.py")],
)
