#ifndef UTIL_LOGGER_H_
#define UTIL_LOGGER_H_

#include <cstdio>

/**
 * @author Patrick Fairbank
 *
 * Class for writing to a log file on the cRIO's filesystem.
 */
class Logger {
 public:
  /**
   * Constructor.
   * @param filename The file path on the cRIO to log to
   * @param loggingInterval Only log once per this many lines to reduce verbosity (optional)
   */
  Logger(const char* filename, int loggingInterval = 1);

  virtual ~Logger();

  /**
   * Writes the given output to the log. Uses printf syntax.
   */
  void Log(const char* format, ...);

 private:
  FILE* logfile_;
  int loggingInterval_;
  int intervalCounter_;
};

#endif  // UTIL_LOGGER_H_
