#include "vision/BackboardFinder.h"

#include "WPILib.h"

double BackboardFinder::GetX() {
  return x_;
}

bool BackboardFinder::SeesTarget() {
  return seesTarget_;
}

void BackboardFinder::DoVision() {
  // Get image from camera
  AxisCamera &camera = AxisCamera::GetInstance("10.2.54.11");
  ColorImage img(IMAQ_IMAGE_RGB);
  if (!camera.GetImage(&img))
    return;
  camera.WriteResolution(AxisCameraParams::kResolution_320x240);

  // RGB Threshold -> BinaryImage
  BinaryImage* bimg = img.ThresholdRGB(0,50,0,50,80,255);

  // take out small things
  Image* image = bimg->GetImaqImage();
  if (!image)
    return;
  int pKernel[9] = {1,1,1,1,1,1,1,1,1};
  StructuringElement structElem;
  structElem.matrixCols = 3;
  structElem.matrixRows = 3;
  structElem.hexa = FALSE;
  structElem.kernel = pKernel;

  // Filters particles based on their size.
  imaqSizeFilter(image, image, TRUE, 1, IMAQ_KEEP_LARGE, &structElem);

  // Convex Hull
  imaqConvexHull(image, image, true);

  // Convex Hull Perimeter, pParamter = perimeter Paramater
  int pParameter[1] = {20};
  double pLower[1] = {0};
  double pUpper[1] = {100};
  int pCalibrated[1] = {0};
  int pExclude[1] = {0};
  
  ParticleFilterCriteria2* pParticleCriteria = NULL;
  ParticleFilterOptions pParticleFilterOptions;
  int pNumParticles;
  
  pParticleCriteria = (ParticleFilterCriteria2*)malloc(sizeof(ParticleFilterCriteria2));
  pParticleCriteria[0].parameter = (MeasurementType)pParameter[0];
  pParticleCriteria[0].lower = pLower[0];
  pParticleCriteria[0].upper = pUpper[0];
  pParticleCriteria[0].calibrated = pCalibrated[0];
  pParticleCriteria[0].exclude = pExclude[0];

  pParticleFilterOptions.rejectMatches = TRUE;
  pParticleFilterOptions.rejectBorder = 0;
  pParticleFilterOptions.connectivity8 = TRUE;
  imaqParticleFilter3(image, image, pParticleCriteria, 1, &pParticleFilterOptions, NULL, &pNumParticles);
  free(pParticleCriteria);
  
  // Filter based on squarishness, eParamter = elongationParamter
  int eParameter =  53;
  double eLower = 1.2;
  double eUpper = 2.7;
  int eCalibrated = 0;
  int eExclude = 0;
  ParticleFilterCriteria2* eParticleCriteria = NULL;
  ParticleFilterOptions eParticleFilterOptions;
  int eNumParticles;
  eParticleCriteria = (ParticleFilterCriteria2*)malloc(sizeof(ParticleFilterCriteria2));

  eParticleCriteria[0].parameter = (MeasurementType) eParameter;
  eParticleCriteria[0].lower = eLower;
  eParticleCriteria[0].upper = eUpper;
  eParticleCriteria[0].calibrated = eCalibrated;
  eParticleCriteria[0].exclude = eExclude;

  eParticleFilterOptions.rejectMatches = FALSE;
  eParticleFilterOptions.rejectBorder = 0;
  eParticleFilterOptions.connectivity8 = TRUE;
  imaqParticleFilter3(image, image, eParticleCriteria, 1, &eParticleFilterOptions, NULL, &eNumParticles);
  free(eParticleCriteria);
  
  // Extract Particles (4?)
  ParticleAnalysisReport left, right, top, bottom;
  vector<ParticleAnalysisReport>* particles = bimg->GetOrderedParticleAnalysisReports();

  // Find L,R,T,B based on CoM X,Y
  int i = 0;
  if (particles->size() == 0) {
    seesTarget_ = false;
  }
  else {
    seesTarget_ = true;
  }
  for (i = 0; i < particles->size(); i++) {
    ParticleAnalysisReport p = (*particles)[i];
    if (i == 0) {
      left = top = right = bottom = p;
    }
    if (p.center_mass_x_normalized < left.center_mass_x_normalized)
      left = p;
    else if (p.center_mass_x_normalized > right.center_mass_x_normalized)
      right = p;
    else if (p.center_mass_y_normalized > top.center_mass_y_normalized)
      top = p;
  }
  //  printf("Left | X: %f | Y: %f | A: %f\n", (float)left.center_mass_x_normalized, (float) left.center_mass_x_normalized, (float) left.particleArea);
  //  printf("Right | X: %f | Y: %f | A: %f\n", (float)right.center_mass_x_normalized, (float) right.center_mass_x_normalized, (float) right.particleArea);

  // Calculate distance

  // Calculate x offset from target center
  x_ = top.center_mass_x_normalized;
  // Calculate angle on fieled based on ?

  delete bimg;
  static double t = 0;
  double diff = Timer::GetFPGATimestamp() - t;
  t = Timer::GetFPGATimestamp();
  lastUpdate_ = t;
  //  printf("Top | X: %f | Y: %f | A: %f | dt: %f\n", (float)top.center_mass_x_normalized, (float) top.center_mass_y_normalized, (float) top.particleArea, 1.0/diff);
  // printf("up: %f Hz | left comx: %f\n", 1.0/diff,(float) left.center_mass_x_normalized);
  double middleGap = right.boundingRect.left - (left.boundingRect.left + left.boundingRect.width);
  printf("tX: %f | Gap: %f | Width: %f | Hz: %f\n", (float)top.center_mass_x_normalized, (float) middleGap, (float) top.boundingRect.width, 1.0/diff);
}

bool BackboardFinder::HasFreshTarget() {
  return (Timer::GetFPGATimestamp() - lastUpdate_ < .5); // 500ms
}
