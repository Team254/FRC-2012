#include "drivers/Driver.h"

#include "subsystems/Drive.h"

Driver::Driver(Drive* drive) {
  drive_ = drive;
}
