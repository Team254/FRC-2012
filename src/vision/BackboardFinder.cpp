#include "vision/BackboardFinder.h"
#include "config/Constants.h"
#include "WPILib.h"

void BackboardFinder::DoVision() {
  Constants* constants = Constants::GetInstance();

  // Get image from camera
  AxisCamera &camera = AxisCamera::GetInstance("10.2.54.90");
  ColorImage img(IMAQ_IMAGE_RGB);
  camera.GetImage(&img);

  // RGB Threshold -> BinaryImage
  Threshold thresh = Threshold(constants->thresholdRMin, constants->thresholdBMax,
                               constants->thresholdGMin, constants->thresholdGMax,
                               constants->thresholdBMin, constants->thresholdRMax);

  BinaryImage* bimg = img.ThresholdRGB(thresh);

  // take out small things

  // Convex Hull

  // filter on shape of partice (only let squarish thing through)

  // Extract Particles (4?)

  // Find L,R,T,B based on CoM X,Y

  // Calculate distance

  // Calculate x offset from target center

  // Calculate angle on fieled based on ?


  static int i = 0;
  x_++;
  DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line1,1,"i: %d" , x_);
  DriverStationLCD::GetInstance()->UpdateLCD();
}

