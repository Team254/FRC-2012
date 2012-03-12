#ifndef UTIL_MOVING_AVERAGE_FILTER_
#define UTIL_MOVING_AVERAGE_FILTER_

#include <deque>

class MovingAverageFilter {
 public:
  MovingAverageFilter(int numTaps);
  double Update(double newV);
  void Reset();

 private:
  std::deque<double> vals_;
  int taps_;
};

#endif
