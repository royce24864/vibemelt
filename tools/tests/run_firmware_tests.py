"""Run the refactored firmware's native regression suites, without the simulator."""
from pathlib import Path
import argparse
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "c++"))
    args = parser.parse_args()
    build = ROOT / "build"
    build.mkdir(parents=True, exist_ok=True)
    compiler = args.compiler
    if Path(compiler).is_file():
        compiler = str(Path(compiler).resolve())
    executable = build / ("config-store-tests.exe" if os.name == "nt" else "config-store-tests")
    command = [compiler]
    if Path(compiler).stem.lower() == "zig":
        command.append("c++")
        if os.name == "nt":
            command += ["-target", "x86_64-windows-gnu"]
    command += ["-std=c++17", "-Wall", "-Wextra", "-Werror",
                str(ROOT / "tools/tests/config_store_tests.cpp"), "-o", str(executable)]
    print("\nConfiguration storage", flush=True)
    subprocess.run(command, cwd=ROOT, check=True)
    subprocess.run([str(executable)], cwd=ROOT, check=True)
    for name in ("control", "adapter", "motor_timing", "dshot", "io"):
        print(f"\n{name.replace('_', ' ').title()}", flush=True)
        subprocess.run([sys.executable, str(ROOT / f"tools/tests/run_{name}_tests.py"),
                        "--compiler", compiler], cwd=ROOT, check=True)
    print("\nAll firmware regression suites passed.", flush=True)


if __name__ == "__main__":
    main()
