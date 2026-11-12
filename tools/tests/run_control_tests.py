"""Compile the real hardware-independent controller, estimator and calibration modules."""
from pathlib import Path
import argparse
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SKETCH = ROOT / "src/main_refactored"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    bundled = ROOT / "build/review-tools/ziglang/zig.exe"
    parser.add_argument("--compiler", default=os.environ.get("CXX", str(bundled) if bundled.exists() else "c++"))
    args = parser.parse_args()
    output = ROOT / "build/control-tests"
    output.mkdir(parents=True, exist_ok=True)
    command = [args.compiler]
    if Path(args.compiler).stem.lower() == "zig":
        command += ["c++"]
        if os.name == "nt":
            command += ["-target", "x86_64-windows-gnu"]
    exe = output / ("firmware-control-tests.exe" if os.name == "nt" else "firmware-control-tests")
    command += ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(SKETCH),
                str(SKETCH / "Controller.cpp"), str(SKETCH / "Estimator.cpp"),
                str(SKETCH / "Calibration.cpp"),
                str(ROOT / "tools/tests/firmware_control_tests.cpp"), "-o", str(exe)]
    subprocess.run(command, check=True, cwd=ROOT)
    subprocess.run([str(exe)], check=True, cwd=ROOT)


if __name__ == "__main__":
    main()
