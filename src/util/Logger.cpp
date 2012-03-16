#include "util/Logger.h"
#include <cstdarg>

Logger* Logger::instance_ = NULL;

Logger::Logger(const char* filename, int loggingInterval) {
  // Open the log file in append mode to preserve any existing contents.
  logfile_ = fopen(filename, "a");
  loggingInterval_ = loggingInterval;
  intervalCounter_ = 0;
  filename_ = filename;
}

Logger::~Logger() {
  fclose(logfile_);
}

void Logger::Log(const char* format, ...) {
  // Only output to the log at the specified interval.
  if (intervalCounter_ == 0) {
    va_list args;
    va_start(args, format);
    printf("Hurr\n");
    vfprintf(logfile_, format, args);
    vprintf(format, args);
    va_end(args);
    fflush(logfile_);
  }

  intervalCounter_++;
  if (intervalCounter_ >= loggingInterval_) {
    intervalCounter_ = 0;
  }
}

void Logger::ClearLog() {
  // Close the log file and reopen it in write mode to overwrite any existing contents.
  logfile_ = freopen(filename_.c_str(), "w", logfile_);
}

Logger* Logger::GetSysLog() {
  if (Logger::instance_ == NULL) {
    Logger::instance_ = new Logger("robot.log");
  }
  return Logger::instance_;
}
