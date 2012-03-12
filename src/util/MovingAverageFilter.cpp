#include "util/MovingAverageFilter.h"

MovingAverageFilter::MovingAverageFilter(int numTaps) {
  taps_ = numTaps;
}

void MovingAverageFilter::Reset() {
  vals_.clear();
}

double MovingAverageFilter::Update(double val) {
  if ((int) vals_.size() >= taps_)
    vals_.pop_front();

  vals_.push_back(val);
  double avg = 0;
  for (int i = 0; i < (int) vals_.size(); i++) {
    avg += vals_[i];
  }
  return avg / ((double)vals_.size());
}
