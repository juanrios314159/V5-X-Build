#pragma once
#include "util.hpp"

void intake(bool reversed);
void lift(bool reversed);
void armToggle();

namespace meth {
    inline constexpr double PI = 3.14159265358979323846; //local declaration. Leads to further preceision rather than using external class

    double toRadians(double deg);
    double toDegrees(double rad);
    double clamp01(double v);

    double wrap360(double deg);
    double wrap180(double deg);

    void headingToUnitVector(double headingDeg, double& dx, double& dy);

    void robotToFieldVector(double x_r, double y_r, double thetaDeg, double& x_f, double& y_f);
    void fieldToRobotVector(double x_f, double y_f, double thetaDeg, double& x_r, double& y_r);
}