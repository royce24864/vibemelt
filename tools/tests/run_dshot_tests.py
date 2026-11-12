"""Compile DShotRMT.cpp directly against controlled RMT hardware stubs."""
from pathlib import Path
import argparse
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--compiler", default=os.environ.get("CXX", "c++"))
args = parser.parse_args()
firmware = ROOT / "src" / "main_refactored"
tests = ROOT / "tools" / "tests"
output = ROOT / "build" / ("dshot-transport-tests.exe" if os.name == "nt" else "dshot-transport-tests")
output.parent.mkdir(parents=True, exist_ok=True)
command = [args.compiler]
if Path(args.compiler).stem.lower() == "zig":
    command += ["c++"]
    if os.name == "nt":
        command += ["-target", "x86_64-windows-gnu"]
command += ["-std=c++17", "-Wall", "-Wextra", "-Wno-unused-parameter",
            "-Wno-missing-field-initializers", "-I", str(tests / "dshot_stubs"),
            "-I", str(firmware), str(firmware / "DShotRMT.cpp"),
            str(tests / "dshot_transport_tests.cpp"), "-o", str(output)]
subprocess.run(command, cwd=ROOT, check=True)
subprocess.run([str(output)], cwd=ROOT, check=True)
