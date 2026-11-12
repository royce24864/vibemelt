"""Compile Motor_Timing.cpp directly; no simulator or source extraction."""
from pathlib import Path
import argparse
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--compiler", default=os.environ.get("CXX", "c++"))
args = parser.parse_args()
firmware = ROOT / "src" / "main_refactored"
output = ROOT / "build" / ("motor-timing-tests.exe" if os.name == "nt" else "motor-timing-tests")
output.parent.mkdir(parents=True, exist_ok=True)
command = [args.compiler]
if Path(args.compiler).stem.lower() == "zig":
    command += ["c++"]
    if os.name == "nt":
        command += ["-target", "x86_64-windows-gnu"]
command += ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(firmware),
            str(firmware / "Motor_Timing.cpp"),
            str(ROOT / "tools" / "tests" / "motor_timing_tests.cpp"), "-o", str(output)]
subprocess.run(command, cwd=ROOT, check=True)
subprocess.run([str(output)], cwd=ROOT, check=True)
