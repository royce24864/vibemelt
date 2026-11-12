"""Exercise src/main direction calibration with fake NVS, without hardware."""
import argparse
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True)
    args = parser.parse_args()
    build = ROOT / "build/main-spin-tests"
    build.mkdir(parents=True, exist_ok=True)
    bodies = []
    for filename, names in {
        "Heading.ino": ["clearCorrectionTable", "sortCorrectionTable",
                        "load_spin_calibration", "save_spin_calibration",
                        "setCorrectionPoint", "getRotationOffsetForRpm",
                        "wrap360", "current_led_heading_phase_deg", "stick_percent_centered"],
        "main.ino": ["update_spin_direction_input", "handle_config_adjustments"],
    }.items():
        source = (ROOT / "src/main" / filename).read_text()
        for name in names:
            match = re.search(r"^[^\n;{}]*\b" + name + r"\([^;]*?^\{\n.*?^\}",
                              source, re.MULTILINE | re.DOTALL)
            if not match:
                raise ValueError(name)
            bodies.append(match[0])
    source = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include "Spin_Calibration.h"
using std::isfinite;
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define DEBUG_LOGF(...) ((void)0)
struct FakeNvs {
  std::map<std::string, double> values;
  float getFloat(const char *key, float fallback) {
    return values.count(key) ? values[key] : fallback;
  }
  int getInt(const char *key, int fallback) {
    return values.count(key) ? values[key] : fallback;
  }
  void putFloat(const char *key, float value) {
    assert(std::string(key).size() <= 15); values[key] = value;
  }
  void putInt(const char *key, int value) { putFloat(key, value); }
} config_store;
bool validStoredFloat(float value, float low, float high) {
  return isfinite(value) && value >= low && value <= high;
}
SpinCalibration spin_calibration[2];
bool reversed = false, rc_status = true, config_mode = false, wifi_mode_active = false;
enum { CONFIG_STAGE_RPM, CONFIG_STAGE_HEADING };
int config_calibration_stage = CONFIG_STAGE_RPM;
int duty[9] = {};
// Calibration tests inject the already normalized spin request at this boundary;
// the drive-tuning suite exercises the real receiver normalization separately.
float spin_throttle_input_percent = 0;
const int CH_SPIN_DIR = 5, CH_SPIN_THROTTLE = 3;
int spin_requested_command(float) { return duty[CH_SPIN_THROTTLE] * 10; }
const int CH_TRANSLATE_LR = 1, stick_deadzone_percent = 8;
const float config_rpm_offset_rate = .04f, config_led_offset_rate_dps = 90;
unsigned long now_ms = 50, last_config_adjust_ms = 0;
unsigned long millis() { return now_ms; }
void handle_config_save_button() {}
float drive_heading_phase_deg = 30;
bool near(float a, float b) { return std::fabs(a - b) < .00001f; }
'''
    source += "\n".join(bodies)
    source += r'''
int main() {
  // Legacy values seed both directions; each is a separate copy.
  config_store.putFloat("rpm_offset", 1.04f);
  config_store.putFloat("led_heading", 300);
  config_store.putInt("corr_count", 2);
  config_store.putFloat("rpm0", 2000); config_store.putFloat("off0", 1.08f);
  config_store.putFloat("rpm1", 1000); config_store.putFloat("off1", 1.02f);
  SpinCalibration defaults;
  load_spin_calibration(spin_calibration[0], "", defaults);
  load_spin_calibration(spin_calibration[1], "r_", spin_calibration[0]);
  assert(near(getRotationOffsetForRpm(1500), 1.05f));
  assert(near(current_led_heading_phase_deg(), 330));
  duty[5] = 100;
  update_spin_direction_input();
  assert(reversed); // Works while throttle is zero, before calibration entry.
  assert(near(getRotationOffsetForRpm(1500), 1.05f));
  config_mode = true;
  duty[5] = 0;
  update_spin_direction_input();
  assert(reversed); // CH5 cannot redirect an active calibration session.
  spin_calibration[1].offset = .94f;
  spin_calibration[1].led_heading_offset_deg = 50;
  assert(near(getRotationOffsetForRpm(1500), .94f));
  assert(near(current_led_heading_phase_deg(), 80));
  duty[1] = 0;
  handle_config_adjustments();
  assert(near(spin_calibration[1].offset, .942f));
  assert(near(spin_calibration[0].offset, 1.04f));
  config_calibration_stage = CONFIG_STAGE_HEADING;
  now_ms += 50;
  handle_config_adjustments();
  assert(near(spin_calibration[1].led_heading_offset_deg, 54.5f));
  assert(near(spin_calibration[0].led_heading_offset_deg, 300));
  spin_calibration[1].led_heading_offset_deg = 50;
  clearCorrectionTable(spin_calibration[1]);
  setCorrectionPoint(1000, .92f, -20, nullptr);
  setCorrectionPoint(2000, 1.0f, 40, nullptr);
  config_calibration_stage = CONFIG_STAGE_HEADING;
  assert(near(getRotationOffsetForRpm(1500), .96f));
  assert(near(spin_calibration[0].offset, 1.04f));
  assert(near(spin_calibration[0].correction_offset[0], 1.02f));
  save_spin_calibration(spin_calibration[0], "");
  save_spin_calibration(spin_calibration[1], "r_");
  spin_calibration[0] = defaults; spin_calibration[1] = defaults;
  load_spin_calibration(spin_calibration[0], "", defaults);
  load_spin_calibration(spin_calibration[1], "r_", spin_calibration[0]);
  config_mode = false;
  assert(near(getRotationOffsetForRpm(1500), .96f));
  assert(near(current_led_heading_phase_deg(), 80));
  assert(spin_calibration[1].correction_throttle[0] == -20);
  duty[3] = 20; update_spin_direction_input(); assert(reversed);
  duty[3] = 19; duty[5] = 50; update_spin_direction_input(); assert(reversed);
  duty[5] = 0; rc_status = false; update_spin_direction_input(); assert(reversed);
  rc_status = true; wifi_mode_active = true; update_spin_direction_input(); assert(reversed);
  wifi_mode_active = false; update_spin_direction_input(); assert(!reversed);
  assert(near(getRotationOffsetForRpm(1500), 1.05f));
  assert(near(current_led_heading_phase_deg(), 330));
  clearCorrectionTable(spin_calibration[0]);
  assert(spin_calibration[1].correction_point_count == 2);
  // Invalid reverse data cannot clear the normal table.
  config_store.putInt("r_corr_count", 99);
  load_spin_calibration(spin_calibration[0], "", defaults);
  load_spin_calibration(spin_calibration[1], "r_", spin_calibration[0]);
  assert(spin_calibration[0].correction_point_count == 2);
  assert(spin_calibration[1].correction_point_count == 0);
  std::puts("PASS: direction selection, calibration isolation, interpolation, heading, legacy migration, NVS round trip and invalid profile isolation");
}
'''
    cpp = build / "tests.cpp"
    cpp.write_text(source)
    exe = build / "tests.exe"
    command = [args.compiler]
    if Path(args.compiler).stem.lower() == "zig":
        command += ["c++", "-target", "x86_64-windows-gnu"]
    command += ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-I",
                str(ROOT / "src/main"), str(cpp), "-o", str(exe)]
    subprocess.run(command, check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
