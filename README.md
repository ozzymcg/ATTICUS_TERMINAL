<<<<<<< HEAD
# ATTICUS TERMINAL

An advanced interactive simulation and design tool for autonomous routines. It allows users to place waypoints and paths on a virtual VEX field, and then simulates realistic physics and generates customizable routine code while providing analytical feedback and detailed logging. Autonomous path files are generated in Lemlib standard format.

## Quickstart
1. Download/Extract: grab the latest ZIP (or copy this folder), then open the extracted `ATTICUS TERMINAL` folder.
2. Build the EXE: double-click `export/build_exe.bat` (it will create a `.venv`, install deps, and run PyInstaller).
3. Run it: open `dist/AtticusTerminal/AtticusTerminal.exe`.
4. To Update:
   - If you cloned with Git: run `git pull`, then double-click `export/build_exe.bat` again.
   - If you used the ZIP: re-download the ZIP, replace the files in this folder (keep your `config.json` / auton files if you?ve customized them), then run `export/build_exe.bat`.

## One-command build (PowerShell)
```powershell
cd "<extracted folder>"
.\export\build_exe.ps1
```
Pass `-Force` to rebuild the virtualenv if you want to refresh dependencies: `.\export\build_exe.ps1 -Force`.

=======
# ATTICUS TERMINAL: OVERHAUL

Official Docs and README coming soon
## Installation
Requires python, git, pip
1. **Clone the Repository:**

   ```bash
   git clone https://github.com/ozzymcg/ATTICUS_TERMINAL.git
   cd ATTICUS_TERMINAL
   ```

2. **Install Dependencies:**

   Ensure you have Python 3 installed. Then install the dependencies:

   ```bash
   pip install -r requirements.txt
   ```

## Running the Program

Start the simulation with:

```bash
python main.py
```

## Update
In Terminal directory
```bash
git fetch origin
git pull --rebase origin main
```
>>>>>>> bfc001ff897bd9a595f6f2409987328af6b668fb

For bug reporting or assistance, contact Austin/Ozzy of 15800A through most public VEX servers.
