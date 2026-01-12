#include "vex.h"

using namespace vex;

// Pass in the devices we want to use
Assembly::Assembly(
    mik::motor outtake_motor,
    mik::motor intake_motor, 
    mik::piston mid_goal_piston,
    mik::piston tongue_piston,
    mik::piston wing_piston
) :
    // Assign the ports to the devices
    outtake_motor(outtake_motor),
    intake_motor(intake_motor),
    mid_goal_piston(mid_goal_piston),
    tongue_piston(tongue_piston),
    wing_piston(wing_piston)
{};

// You want to call this function once in the user control function in main.
void Assembly::init() {
    assembly.outtake_motor.setBrake(hold);
    tongue_piston.close();
    mid_goal_piston.close();
    wing_piston.open();
    // Create the task to move the lift arm. We only want one task to be created
    // lift_task = vex::task([](){
    //     assembly.move_lift_arm();
    //     return 0;
    // });
    // To stop the task do `assembly.lift_task.stop();`
} 

// You want to put this function inside the user control loop in main.
void Assembly::control() {
    intake_motors_control();
    mid_goal_piston_control();
    tongue_piston_control();
    wing_piston_control();
}

// Spins intake forward if L1 is being held, reverse if L2 is being held; stops otherwise
void Assembly::intake_motors_control() {
    if (Controller.ButtonL1.pressing()) {
        intake_motor.spin(fwd, 12, volt);
    } else if (Controller.ButtonL2.pressing()) {
        intake_motor.spin(fwd, 12, volt);
        outtake_motor.spin(fwd, 12, volt);
    } else if (Controller.ButtonR1.pressing()) {
        intake_motor.spin(fwd, -12, volt);
        outtake_motor.spin(fwd, -12, volt);
    } else {
        intake_motor.stop();
        outtake_motor.stop();
    }
}


// Extends or retracts piston when button A is pressed, can only extend or retract again until button A is released and pressed again
void Assembly::tongue_piston_control() {
    if (btnB_new_press(Controller.ButtonB.pressing())) {
        tongue_piston.toggle();
    }
}

// Extends or retracts piston when button A is pressed, can only extend or retract again until button A is released and pressed again
void Assembly::wing_piston_control() {
    if (btnR2_new_press(Controller.ButtonR2.pressing())) {
        wing_piston.toggle();
    }
}

// Extends or retracts piston when button A is pressed, can only extend or retract again until button A is released and pressed again
void Assembly::mid_goal_piston_control() {
    if (btnDown_new_press(Controller.ButtonDown.pressing())) {
        mid_goal_piston.toggle();
    }
}