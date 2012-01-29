#include "util/Logger.h"

#include <cstdarg>

Logger::Logger(const char* filename, int loggingInterval) {
  logfile_ = fopen(filename, "a");
  loggingInterval_ = loggingInterval;
  intervalCounter_ = 0;
}

Logger::~Logger() {
  fclose(logfile_);
}

void Logger::Log(const char* format, ...) {
  // Only output to the log at the specified interval.
  if (intervalCounter_ == 0) {
    va_list args;
    va_start(args, format);
    vfprintf(logfile_, format, args);
    va_end(args);
    fflush(logfile_);
  }

  intervalCounter_++;
  if (intervalCounter_ >= loggingInterval_) {
    intervalCounter_ = 0;
  }
}
