#include "main.h"
#include "api.h"
#include "globals.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include "liblvgl/lvgl.h"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
// #include "robodash/core.h"
// #include "robot/monte.hpp"
#include "robot/auton.h"
#include "robot/skills.h"
#include <iostream>
#include "util.hpp"

using namespace pros;
void initialize() {
    lcd::initialize(); //screen turns on
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);//sets drivetrain to brake so auton doesnt drift
	liftMotors.set_brake_mode(E_MOTOR_BRAKE_HOLD); //lift must hold its position
	scoreArm.set_brake_mode(E_MOTOR_BRAKE_HOLD); //arm must hold its position

    chassis.calibrate();    // Start calibration
    while (imu.is_calibrating()) delay(10); //doesnt advance anything until calibration is done to avoid errors

    chassis.setPose(0, 0, 0);    // Set initial pose
	liftRotation.set_position(0);// sets default down lift position
	armRotation.set_position(0); //sets default down arm postion

    delay(10); //delay for courtesy

	// Background LCD task 
	static Task screenTask([] {
		while (true) { //forever, display the x,y,theta coordinates of the robot
			liftPosition = liftRotation.get_position();//continously update lift position to use in util.cpp
			armPosition = armRotation.get_position();//countinously update armposition to use in util.cpp
			chassisX = chassis.getPose().x; //continously update chassis X position 
			chassisY = chassis.getPose().y; //continously update chassis Y position
			chassisTheta = chassis.getPose().theta; //conitinously update chassis theta position
			lcd::print(0, "X: %.2f", chassisX); //x
			lcd::print(1, "Y: %.2f", chassisY); //y
			lcd::print(2, "Theta: %.2f", chassisTheta); //angle
			lcd::print(3, "liftRotation: %.2f", liftPosition); //lift angle
			lcd::print(4, "armRotation: %.2f", armPosition); //arm angle
			//tuning distance sensors later on
      		// Print distance sensor values in inches
			// pros::lcd::print(4, "N: %.2f in", dNorth.get_distance() / 25.4);
			// pros::lcd::print(5, "E: %.2f in", dEast.get_distance() / 25.4);
			// pros::lcd::print(6, "S: %.2f in", dNorthW.get_distance() / 25.4);
			// pros::lcd::print(7, "W: %.2f in", dWest.get_distance() / 25.4);

			pros::delay(60); //delay to limit resource usage
		}
	});
}

void disabled() {}
void competition_initialize() {
}
void autonomous() {}
void opcontrol() {
	chassis.setBrakeMode(E_MOTOR_BRAKE_COAST); //ts so we can go ZOOOOOOOM
	Controller master(pros::E_CONTROLLER_MASTER); //declar controller so we can take intput

	while(true){
        double forward = master.get_analog(ANALOG_LEFT_Y); //left joystick defines foreward and background
        double turn = master.get_analog(ANALOG_RIGHT_X); //right joystick defines turning
        chassis.arcade(forward, turn); //make it arcade tank drive
		
		//internal rollers
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1))
			intake(false);//intake
		else if (master.get_digital(E_CONTROLLER_DIGITAL_L2))
			intake(true);//score
	
		//Lift
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1))
			lift(false);//lift Up
		else if (master.get_digital(E_CONTROLLER_DIGITAL_R2))
			lift(true);//Bring Down

		if(master.get_digital(E_CONTROLLER_DIGITAL_A))
			armToggle;//fold arm

		delay(20);// limit ts to not fry le brain
	}

}
