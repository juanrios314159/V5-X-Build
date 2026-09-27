#pragma once

#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

extern pros::MotorGroup left_DT;
extern pros::MotorGroup right_DT;
extern lemlib::Drivetrain drivetrain;

extern pros::MotorGroup intakeMotors;
extern pros::MotorGroup liftMotors;
extern pros::Motor scoreArm;
extern bool armDown;

extern pros::Imu imu;
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
