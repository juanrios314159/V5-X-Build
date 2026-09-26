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
    if(abs(liftPosition) < 10 && armDown == true)//detects if lift is down +-10degrees AND arm is down
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
        }
    } 
    if((!armDown)&&abs(armPosition)<10){ //ONLY if the arm is extended AND it is at starting postition +-10degrees, will it retract
        while(armPosition > armDownPosition){//limiter to go down to zero
            scoreArm.move(-127);
        }
    }
    armDown = !armDown;//inverts arm tracking boolean
}