#pragma once

#include "vex.h"

using namespace vex;

class Assembly {
public:
    Assembly(
        mik::motor outtake_motor,
        mik::motor intake_motor, 
        mik::piston tongue_piston,
        mik::piston wing_piston,
        mik::piston mid_goal_piston,
        mik::piston ptoPiston
    );

    enum ptoMode{
        intakeOnly,
        both
    };

    ptoMode pto_mode = intakeOnly;

    
    void init();
    void control();

    void intake_motors_control();
    void tongue_piston_control();
    void wing_piston_control();
    void mid_goal_piston_control();
    void pto_control();
    
    mik::motor outtake_motor;
    mik::motor intake_motor;
    mik::piston tongue_piston;
    mik::piston wing_piston;
    mik::piston mid_goal_piston;
    mik::piston ptoPiston;
};