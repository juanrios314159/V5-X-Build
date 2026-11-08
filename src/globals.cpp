#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include "robot/auton.h"
#include "globals.h"

// define namespace bc lazy
using namespace pros; // ion wanna type pros before everything :D

// HARDWARE DECLARATION

// Drivetrain
MotorGroup left_DT({10, 9}, MotorGearset::blue);    // Defines left drive train motors
MotorGroup right_DT({8, 7}, MotorGearset::blue);  // Defines right drive train motors
lemlib::Drivetrain drivetrain(&left_DT, &right_DT, 12.25 /*adjust for width*/, 3.25, 450, 2);

// Got that intake 
MotorGroup intakeMotors({5, 6});  

//Lift
MotorGroup liftMotors({2, 3}, MotorGearset::red);  // Defines lift motors
Motor scoreArm(4);
bool armDown = true;
// Sensors


OffsetIMU imu(21);

Rotation liftRotation(1); 
float liftPosition;
Rotation armRotation(11);
float armPosition;
//Declaration of tracking wheels for odometry. Adjust when add.
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// PID settings --- Still need to tune 
lemlib::ControllerSettings lateralController(//tune pid
    16.0,
    0.0,
    25.0,
    0.0,
    0.5,
    80.0,
    0.0,
    0.0,
    3.0
);

lemlib::ControllerSettings angularController(//tune pid.
    2.275,
    0.0,
    21.0,
    0.0,
    0.5,
    80.0,
    0.0,
    0.0,
    3.0
);

// Drive curves
lemlib::ExpoDriveCurve throttleCurve(2, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(2, 10, 1.019);

// Chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

//uninitialized until define starting position in main
float chassisX;
float chassisY;
float chassisTheta;

//sensors for mcl
pros::Distance dNorth(13);
pros::Distance dEast(14);
pros::Distance dSouth(15);
pros::Distance dWest(16);


//GUI
std::function<void()> selected_auton      = run_do_nothing;
std::string           selected_auton_name = "Do Nothing";
DriverSettings driver_settings{};
double battery_percent  = 0.0;
double imu_heading_deg  = 0.0;
