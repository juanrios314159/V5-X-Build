#pragma once
#include <cmath>
#include <algorithm>

void intake(bool reversed);
void lift(bool reversed);
void armToggle();

namespace meth{
    constexpr double PI = 3.14159265358979323846;

    inline double toRadians(double deg) { return deg * PI / 180.0; }
    inline double toDegrees(double rad) { return rad * 180.0 / PI; }
    inline double clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }

    double wrap360(double deg);
    double wrap180(double deg);
    
    void headingToUnitVector(double headingDeg, double& dx, double& dy);
    
    void robotToFieldVector(double x_r, double y_r, double thetaDeg, double& x_f, double& y_f);
    void fieldToRobotVector(double x_f, double y_f, double thetaDeg, double& x_r, double& y_r);
}
