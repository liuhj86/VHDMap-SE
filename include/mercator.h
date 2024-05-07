#ifndef VHDMAPSE_INCLUDE_MERCATOR_H_
#define VHDMAPSE_INCLUDE_MERCATOR_H_

#include <cmath>

class MercatorProj {
 public:
  MercatorProj();

  void SetAB(double& a, double& b);
  void SetB0(double b0);
  void SetL0(double l0);
  int FromProj(double X, double Y, double& B, double& L);
  int ToProj(double B, double L, double& X, double& Y);

  ~MercatorProj();

  int __IterativeTimes;     // The number of iterations in reverse conversion
                            // programs
  double __IterativeValue;  // Iteration initial value in reverse conversion
                            // program
  double __A;               
  double __B;
  double __B0;              // Standard latitude, radians
  double __L0;              // Standard longitude, radians
};
#endif  // VHDMAPSE_INCLUDE_MERCATOR_H_
