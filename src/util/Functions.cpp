#include "util/functions.h"

#include <cmath>

#define PI 3.14159265

double Functions::SquareWave(double timeSec, double periodSec, double amplitude) {
  int periodMs = (int)(periodSec * 1000);
  int incrementTimeMs = (int)(timeSec * 1000) % periodMs;
  if (incrementTimeMs < periodMs / 2) {
    return amplitude;
  } else {
    return -amplitude;
  }
}

double Functions::SineWave(double timeSec, double periodSec, double amplitude) {
  return sin(timeSec * 2 * PI / periodSec) * amplitude;
}
