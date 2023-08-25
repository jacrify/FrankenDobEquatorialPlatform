#ifndef PLATFORM_MODEL_H
#define PLATFORM_MODEL_H




class PlatformModel {
public:
  int calculateFowardSpeedInMilliHz(double distanceFromCenterInMM);
  double getGreatCircleRadius();
  void setGreatCircleRadius(double r);
  double getStepsPerMM();
  int getMiddlePosition();
  int getLimitPosition();
};

#endif