#include "robot/skills.h"
#include "api.h"
#include "globals.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include <random>

void driverSkills(){
}


void progSkills(){
}


std::vector<AutonOption> skills_options = {
    {"Driver Skills",      driverSkills},
    {"Programming Skills", progSkills},
};
