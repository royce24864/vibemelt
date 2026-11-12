"""Exercise firmware orchestration with bounded fake RC, sensor, clock and motor I/O.

The controller/estimator/calibration are compiled directly. Only Arduino adapter
functions are collected from sketch tabs; their bodies are never reimplemented.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SKETCH = ROOT / "src/main_refactored"


def extract(source, name):
    masked = re.sub(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', lambda m: " " * len(m[0]), source)
    match = re.search(r'^[\w :*&<>]+\b' + name + r'\s*\([^;{}]*\)\s*\{', masked, re.MULTILINE)
    if not match:
        raise ValueError("Cannot locate production adapter: " + name)
    begin = match.start()
    opening = masked.index("{", begin)
    end, depth = opening + 1, 1
    while depth:
        depth += (masked[end] == "{") - (masked[end] == "}")
        end += 1
    return source[begin:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "c++"))
    args = parser.parse_args()
    output = ROOT / "build/adapter-tests"
    output.mkdir(parents=True, exist_ok=True)
    selections = {
        "Config_Store.ino": ["capture_runtime_config", "apply_runtime_config"],
        "Motor_Control.ino": ["robot_state_allows_motor_output", "motor_output_gate_open"],
        "RC_Handler.ino": ["set_safe_channels", "rc_frame_values_valid", "update_channels"],
        "Heading.ino": ["record_accel_loss", "calcrpm", "get_accel_force_g", "stick_percent_centered", "accel_zero_cancel_reason", "set_accel_zero_offset"],
        "main_refactored.ino": ["current_controls", "current_control_health", "current_calibration_safety", "latch_control_rearm", "update_control_rearm", "update_robot_state", "state_requires_zero_motor_demand", "stop_for_current_state", "config_entry_controls_safe", "stop_config_motors", "config_abort_reason", "config_exit_reason_name", "exit_config_mode", "enter_config_mode", "update_config_mode", "update_spin_direction_input", "loop"],
    }
    bodies = []
    for filename, names in selections.items():
        source = (SKETCH / filename).read_text()
        bodies.extend(extract(source, name) for name in names)
    (output / "adapter_prototypes.inc").write_text("\n".join(body[:body.index("{")].strip() + ";" for body in bodies))
    (output / "adapters_under_test.inc").write_text("\n".join(bodies))
    command = [args.compiler]
    if Path(args.compiler).stem.lower() == "zig":
        command += ["c++"]
        if os.name == "nt": command += ["-target", "x86_64-windows-gnu"]
    exe = output / ("firmware-adapter-tests.exe" if os.name == "nt" else "firmware-adapter-tests")
    command += ["-std=c++17", "-I", str(SKETCH), "-I", str(output)]
    command += [str(SKETCH / name) for name in ("Controller.cpp", "Estimator.cpp", "Calibration.cpp", "Motor_Timing.cpp")]
    command += [str(ROOT / "tools/tests/firmware_adapter_tests.cpp"), "-o", str(exe)]
    subprocess.run(command, check=True, cwd=ROOT)
    subprocess.run([str(exe)], check=True, cwd=ROOT)


if __name__ == "__main__": main()
