#ifndef UTIL_MOVING_AVERAGE_FILTER_
#define UTIL_MOVING_AVERAGE_FILTER_

#include <deque>

/**
 * Filters data according to a moving average.
 *
 * A moving average filter outputs the average of the last n inputs given
 * to the filter. This tends to "smooth" out data given to the filter.
 *
 * A larger value of n gives an average that "lags" more, but a smaller value
 * is more sensitive to noise.
 * 
 * This class simultaneously takes a data point to be filtered and returns the
 * current moving average using the method Update().
 * @author Tom Bottiglieri
 */
class MovingAverageFilter {
 public:
  /**
   * Creates a moving average filter which applies to the last given number
   * of data inputs.
   * @param numTaps the number of data points to apply the average to.
   */
  MovingAverageFilter(int numTaps);
  
  /**
   * Pushes an input value into the filter and returns the next updated output 
   * moving average.
   *
   * If the number of data points entered is less than the number of inputs
   * measured by the moving average, a partial moving average will return 
   * which consists only of the data currently entered.
   * @param newV the value to push into the filter
   * @return the current moving average
   */
  double Update(double newV);

  /**
   * Resets the moving average filter, clearing all previously given data
   * points.
   */
  void Reset();

 private:
  std::deque<double> vals_;
  int taps_;
};

#endif
