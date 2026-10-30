/* 
// This file is subject to the terms and conditions defined in
// file 'LICENSE.md', which is part of this source code package.
*/

#ifndef _RF_COMPONENT_COLOR_DETECTION_H_
#define _RF_COMPONENT_COLOR_DETECTION_H_

#include "Arduino.h"

#define COLOR_DETECT_OK (0)
#define COLOR_DETECT_ERR (1)

typedef struct _color_definition_t {
  double min_mag;
  double max_mag;
  double hue;
  double min_hue;
  double max_hue;
  const char* name;
  size_t staff_val;
} color_definition_t;

typedef struct _rgb_f_t {
  double r = 0.0;  // [0.0, 1.0]
  double g = 0.0;  // [0.0, 1.0]
  double b = 0.0;  // [0.0, 1.0]
} rgb_f_t;

typedef struct _hsv_f_t {
  double h = 0.0;  // [0.0, 360.0]
  double s = 0.0;  // [0.0, 1.0]
  double v = 0.0;  // [0.0, 1.0]
} hsv_f_t;

double rgbElemByUI16(uint16_t val);

hsv_f_t rgbToHsv(const rgb_f_t* rgb);
rgb_f_t hsvToRgb(const hsv_f_t* hsv);
double rgbToMagnitude(const rgb_f_t* rgb);

bool colorIndexByRGB(const rgb_f_t* rgb, size_t* idx, const color_definition_t* colors, size_t num_colors);

#endif // _RF_COMPONENT_COLOR_DETECTION_H_
