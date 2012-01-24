#include "vision/BackboardFinder.h"
#include "config/Constants.h"
#include "WPILib.h"

void BackboardFinder::DoVision() {
  Constants* constants = Constants::GetInstance();

  AxisCamera &camera = AxisCamera::GetInstance("10.2.54.90");
  ColorImage img(IMAQ_IMAGE_RGB);
  camera.GetImage(&img);
  Threshold thresh = Threshold(constants->thresholdHMin, constants->thresholdHMax,
                               constants->thresholdSMin, constants->thresholdSMax,
                               constants->thresholdVMin, constants->thresholdVMax);
  BinaryImage* bimg = img.ThresholdHSV(thresh);

  static int i = 0;
  i++;
  DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line1,1,"i: %d" , i);
  DriverStationLCD::GetInstance()->UpdateLCD();
}

