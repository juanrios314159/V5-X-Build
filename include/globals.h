#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "util.hpp"

extern pros::MotorGroup left_DT;
extern pros::MotorGroup right_DT;
extern lemlib::Drivetrain drivetrain;

extern pros::MotorGroup intakeMotors;
extern pros::MotorGroup liftMotors;
extern pros::Motor scoreArm;
extern bool armDown;

class OffsetIMU : public pros::Imu {
    public:
        OffsetIMU(uint8_t port) : pros::Imu(port) {}
        double get_heading() {
            return meth::wrap360(pros::Imu::get_heading() - 90.0);
        }
        double get_rotation() {
            return pros::Imu::get_rotation() - 90.0;
        }
};

extern OffsetIMU imu;
extern pros::Rotation liftRotation;
extern float liftPosition;
extern pros::Rotation armRotation;
extern float armPosition;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateralController;
extern lemlib::ControllerSettings angularController;

extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

extern lemlib::Chassis chassis;
extern float chassisX;
extern float chassisY;
extern float chassisTheta;

extern pros::Distance dNorth;
extern pros::Distance dEast;
extern pros::Distance dSouth;
extern pros::Distance dWest;

struct AutonOption {
    const char*        name;
    std::function<void()> func;
};
 
enum class DriveMode {
    ARCADE,
    TANK
};
 
enum class BrakeMode {
    COAST,
    BRAKE,
    HOLD
};
 
struct DriverSettings {
    DriveMode drive_mode  = DriveMode::ARCADE;
    BrakeMode brake_mode  = BrakeMode::COAST;
};
 
inline pros::motor_brake_mode_e_t to_pros_brake(BrakeMode mode) {
    switch (mode) {
        case BrakeMode::BRAKE: return pros::E_MOTOR_BRAKE_BRAKE;
        case BrakeMode::HOLD:  return pros::E_MOTOR_BRAKE_HOLD;
        case BrakeMode::COAST:
        default:                return pros::E_MOTOR_BRAKE_COAST;
    }
}
 
extern DriverSettings driver_settings;
 
extern std::function<void()> selected_auton;
extern std::string           selected_auton_name;
 
extern double battery_percent;
extern double imu_heading_deg;
