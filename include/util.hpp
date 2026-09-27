#pragma once
#include <cmath>
#include <algorithm>

void intake(bool reversed);
void lift(bool reversed);
void armToggle();

namespace meth{
    extern const double PI;

    inline double toRadians(double deg);
    inline double toDegrees(double rad);
    inline double clamp01(double v);

    double wrap360(double deg);
    double wrap180(double deg);
    
    void headingToUnitVector(double headingDeg, double& dx, double& dy);
    
    void robotToFieldVector(double x_r, double y_r, double thetaDeg, double& x_f, double& y_f);
    void fieldToRobotVector(double x_f, double y_f, double thetaDeg, double& x_r, double& y_r);
}
