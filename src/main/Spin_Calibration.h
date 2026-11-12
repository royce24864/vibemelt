#pragma once

const int MAX_CORRECTION_POINTS = 8;

// Normal and reverse spin keep independent calibration, including staged edits.
struct SpinCalibration
{
  float offset = 1.0f;
  float led_heading_offset_deg = 315.0f;
  int correction_point_count = 0;
  float correction_rpm[MAX_CORRECTION_POINTS] = {};
  float correction_offset[MAX_CORRECTION_POINTS] = {};
  int correction_throttle[MAX_CORRECTION_POINTS] = {};
};
