#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot/auton.h"
#include <cmath>
#include <iostream>
#include <string>

//add autonomous code version thingy something

void auton1(){
}
void auton2(){
}
void run_do_nothing() {
}
std::vector<AutonOption> auton_options = {
    {"Auton 1",   auton1},
    {"Auton 2",  auton2},
};
 
