#include "util.hpp"
#include "globals.h"
#include <algorithm>
#include <cmath>

using namespace pros;
int armUpPostiion = 90; //should rotate 90degrees. Adjust for ratio
int armDownPosition = 0;// should return to og down position when not active
int liftDownPostion = 0;// return to og down position
int liftUpPosition = 0;//test to degree when lift is fully extended
void intake(bool reversed){ //claw and intake move together to intake and score
    intakeMotors.move(reversed ? 127:-127);
}
void lift(bool reversed){
    if(fabs(liftPosition) < 10 && armDown == true)//detects if lift is down +-10degrees AND arm is down
        armToggle();//makes sure arm extended when going up
    if (reversed && (liftPosition > liftDownPostion)){ //only goes down until winch unspools
        liftMotors.move_velocity(-100);
    }else if(!reversed/* && (liftPosition < liftUpPosition)*/){//extends indefinately. Should fix with max extended degree but need to test
        liftMotors.move_velocity(100);
    }else{
        liftMotors.move_velocity(0);//specifcally resets to 0 when lift is fully down
    }
}
void armToggle(){
    if(armDown){
        while(armPosition < armUpPostiion){//extends until it hits the 90 degrees(testing needed)
            scoreArm.move(127);
            delay(10);
        }
        armDown = !armDown;//inverts arm tracking boolean
    } 
    if((!armDown)&&fabs(liftPosition)<10){ //ONLY if the arm is extended AND it is at starting postition +-10degrees, will it retract
        while(armPosition > armDownPosition){//limiter to go down to zero
            scoreArm.move(-127);
            delay(10);

        }
        armDown = !armDown;//inverts arm tracking boolean
    }
}
namespace meth{

    const double PI = 3.14159265358979323846; //local declaration. Leads to further preceision rather than using external class

    double toRadians(double deg) { return deg * PI / 180.0; }//degrees to radians
    double toDegrees(double rad) { return rad * 180.0 / PI; }//radians to degrees

    //wraps degree to usable inclusive 0 and exclusive 360(since 360=0)
    double wrap360(double deg) {
        double r = std::fmod(deg, 360.0); //float mod for precision and finds remainder. 360 equals a rotatotion, so find the extra
        if (r < 0) r += 360.0; //ensures value is positive
        return r;
    }

    // Wrap degree to usable +-180 inclusive
    double wrap180(double deg) {
        double r = std::fmod(deg + 180.0, 360.0); //limit to a semi circle range of view. prevents values from being outside of range by adding 180
        if (r < 0) r += 360.0; //prevents negatives
        return r - 180.0; // takes away the 180 added before
    }

    //yields values equivalent to working with unit circle
    void headingToUnitVector(double headingDeg, double& dx, double& dy) { //defines vector towards which bot is heading.
        double rad = toRadians(headingDeg); //converts to radians for use with sin and cos
        dx = std::sin(rad);
        dy = std::cos(rad);
    }

    // defines robot frame vector and interactino in the field
    void robotToFieldVector(double x_r, double y_r, double thetaDeg, double& x_f, double& y_f) {
        double rad = toRadians(thetaDeg); //convert degrees to radians for use with sin and cos
        double c = std::cos(rad), s = std::sin(rad);
        x_f = x_r * c + y_r * s; //same as (robot frame right) * cos(heading) + (robot frame foreward) * sin(heading)
        y_f = -x_r * s + y_r * c; //same thing as above but (robot frame right direction) is inverted and sin and cos are swapped
    }

    // Inverse of the above. field frame vector -> robot frame vector.
    void fieldToRobotVector(double x_f, double y_f, double thetaDeg, double& x_r, double& y_r) {
        double rad = toRadians(thetaDeg);
        double c = std::cos(rad), s = std::sin(rad);
        x_r = x_f * c - y_f * s;
        y_r = x_f * s + y_f * c;
    }

    double clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }//return value only if 0<x<1. too high: returns 1, negative returns 0. 
}  // namespace

