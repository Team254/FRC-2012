#include "vision/BackboardFinder.h"
#include "config/Constants.h"
#include "WPILib.h"

void BackboardFinder::DoVision() {
  Constants* constants = Constants::GetInstance();

  // Get image from camera
  AxisCamera &camera = AxisCamera::GetInstance("10.2.54.90");
  ColorImage img(IMAQ_IMAGE_RGB);
  if (!camera.GetImage(&img))
    return;

  // RGB Threshold -> BinaryImage
  BinaryImage* bimg = img.ThresholdRGB(0,50,0,50,80,255);
  img.Write("c_img.jpg");
  bimg->Write("t_img.jpg");
  printf(" *********************  Wrote an image to disk  *******************\n");
  // take out small things

  // Convex Hull

  // filter on shape of partice (only let squarish thing through)

  // Extract Particles (4?)

  // Find L,R,T,B based on CoM X,Y

  // Calculate distance

  // Calculate x offset from target center

  // Calculate angle on fieled based on ?

  delete bimg;
}

