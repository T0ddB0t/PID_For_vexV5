#include "vex.h"

class PID{
private:
  double* uT = 0;
public:
  double* Kp = 0;
  double* Ki = 0;
  double* Kd = 0;
  void pidFWBK(double, int);
  void pidTURN(double, int);
};
