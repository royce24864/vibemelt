"""Compile native regressions against the actual refactored firmware bodies.

Usage: python tools/tests/run_io_tests.py --compiler /path/to/zig[.exe]
       python tools/tests/run_io_tests.py --compiler c++

Only hardware/network/RTOS boundaries are mocked. Extracted source is generated
under build/ so edits to firmware are exercised on every invocation.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "src" / "main_refactored"


def extract(source: str, path: Path, pattern: str) -> str:
    match = re.search(pattern, source, re.MULTILINE | re.DOTALL)
    if match is None:
        raise ValueError(f"Cannot locate firmware definition in {path}: {pattern}")
    line = source[: match.start()].count("\n") + 1
    return f'#line {line} "{path.as_posix()}"\n{match.group(0)}\n'


def functions(path: Path, names: list[str]) -> str:
    source = path.read_text(encoding="utf-8")
    # Arduino top-level functions close in column zero. Nested statements do
    # not, so this preserves complete bodies without rewriting their logic.
    return "\n".join(
        extract(source, path, rf"^[^\n;{{}}]*\b{re.escape(name)}\([^;]*?^\{{\n.*?^\}}")
        for name in names
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "c++"))
    args = parser.parse_args()
    build = ROOT / "build" / "firmware-io-tests"
    build.mkdir(parents=True, exist_ok=True)

    led_path = FIRMWARE / "LED_Control.ino"
    led_source = led_path.read_text(encoding="utf-8")
    led = extract(led_source, led_path,
                  r"^static portMUX_TYPE phase_led_state_mux[^;]*;")
    led += extract(led_source, led_path,
                   r"^static PhaseLedSnapshot phase_led_state = \{.*?^\};")
    led += functions(led_path, [
        "read_phase_led_snapshot", "set_phase_led_mode", "publish_phase_led_frame",
        "phase_led_frame_contains_heading", "phase_led_output_task",
    ])
    (build / "firmware_io_led.inc").write_text(led, encoding="utf-8")

    logger = functions(FIRMWARE / "Session_Logger.ino", [
        "session_normalized_channel", "session_acceleration_ceiling",
        "session_acceleration_limiter_active", "session_scaled_motor_average",
        "session_insert_sample", "session_update_summary", "session_capture_current_sample",
        "service_session_logger", "session_logger_resume", "session_state_name",
        "session_oldest_slot", "session_sample_at", "session_sequence_at",
        "session_compute_window_summary",
        "session_handle_timeline_csv", "session_handle_timeline_json",
    ])
    (build / "firmware_io_logger.inc").write_text(logger, encoding="utf-8")

    executable = ROOT / "build" / ("firmware-io-tests.exe" if os.name == "nt"
                                    else "firmware-io-tests-native")
    command = [args.compiler]
    if Path(args.compiler).stem.lower() == "zig":
        command += ["c++"]
        if os.name == "nt":
            command += ["-target", "x86_64-windows-gnu"]
    command += ["-std=c++17", "-Wall", "-Wextra", "-Wno-sign-compare",
                "-I", str(FIRMWARE), "-I", str(build),
                str(FIRMWARE / "Timeline_Cursor.cpp"),
                str(ROOT / "tools" / "tests" / "firmware_io_tests.cpp"),
                "-o", str(executable)]
    subprocess.run(command, cwd=ROOT, check=True)
    subprocess.run([str(executable)], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
