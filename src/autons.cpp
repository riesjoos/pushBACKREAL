#include "vex.h"

using namespace vex;
using namespace mik;

void default_constants(void) {
    chassis.set_control_constants(5, 10, 1.019, 5, 10, 1.019);

    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
	chassis.set_drive_constants(10,1.65,.024,9.87,3);
	chassis.set_turn_constants(12, 0.17, 0.008, 1.3, 2);
    chassis.set_heading_constants(8, .188, 0.006, 1.285, 0.06);
    chassis.set_swing_constants(11, 0.335, 0, 2.3, 0);

    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_turn_exit_conditions(1.5, 75, 2000);
    chassis.set_drive_exit_conditions(1, 75, 3000);
    chassis.set_swing_exit_conditions(1.25, 75, 3000);
}

void odom_constants(void) {
    default_constants();
    chassis.heading_max_voltage = 10;
    chassis.drive_max_voltage = 8;
    chassis.drive_settle_error = 3;
    chassis.boomerang_lead = .5;
    chassis.boomerang_setback = 2;    
}

std::string template_auto(bool calibrate, auto_variation var, bool get_name) {
    /* The first variation will be this auto */
    if (var == one) {}

    /* We declare and allow a second variation of this auto; 
    You may want this if you want a different movements in the same starting configuration */
    if (var == two) { return template_auto_other_variation(calibrate, get_name); }

    if (get_name) { /* Give a desciption of your auto */ return "template auto 1 (3 objs)"; }
    if (calibrate) {
        /* Initialize robots starting position "https://path.jerryio.com/" and/or add extra movements to line up robots 
        starting position **IF MOVING DURING CALIBRATION DO BEFORE FIELD CONTROLLER PLUG IN** */
        chassis.set_coordinates(55, 23.5, 90);
    
        /* Example of turning before auto is ran */
        chassis.turn_max_voltage = 6; 
        chassis.turn_to_angle(45);

        return "";
    }
    
    /* We now run the auto */ 
    chassis.drive_distance(10);
    chassis.drive_distance(-10);

    return "";
}
std::string template_auto_other_variation(bool calibrate, bool get_name) {
    if (get_name) { return "template auto 2 (4 objs)"; }
    
    // Mirror template_auto() from the x-axis
    chassis.mirror_all_auton_y_pos();
    
    if (calibrate) {
        // Coordinates will be set to (55, -23.5) as y_pos is mirrored
        template_auto(calibrate, one, get_name);
        return "";
    }
    
    // Run auto, make sure to pass in one as var.
    template_auto(calibrate, one, get_name);

    return "";
}


std::string blue_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_left_elim(bool calibrate, auto_variation var, bool get_name) {   
    if (get_name) { return "blue left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right elim"; }
    if (calibrate) {
        chassis.set_coordinates(39.709, 6.756, 270);

        return "";
    }

    odom_constants();

    // get cluster
    assembly.intake_motor.spin(forward,12,volt);
    turn_to_point_params p_turn_point = {};
    p_turn_point.timeout = 200;
    chassis.turn_to_point(23,20,p_turn_point);
    chassis.drive_to_point(34.4,17);
    assembly.tongue_piston.toggle();
    drive_distance_params p_drive_dist = {};
    p_drive_dist.max_voltage = 5;
    chassis.drive_distance(10);

    // get single block
    chassis.turn_to_point(11.4,39.2);
    assembly.tongue_piston.toggle();
    drive_to_point_params p_drive_point = {};
    p_drive_point.max_voltage = 8;
    chassis.drive_to_point(11.4,39.2,p_drive_point);
    assembly.tongue_piston.toggle();
    chassis.drive_distance(-20,p_drive_dist);

    // go to score high goal
    chassis.drive_to_point(47.2,44);
    chassis.turn_to_point(57.2,46.7);
    chassis.drive_to_point(30.662,46.699);
    p_drive_dist = {};
    p_drive_dist.timeout = 1000;
    p_drive_dist.max_voltage = 4;
    chassis.drive_distance(-30,p_drive_dist);
    assembly.outtake_motor.spin(forward,12,volt);
    wait(1,sec);
    assembly.outtake_motor.stop();

    chassis.turn_to_point(56.238,46.389);
    p_drive_point = {};
    p_drive_point.max_voltage = 6;
    chassis.drive_to_point(56.238,46.389,p_drive_point);
    p_drive_dist = {};
    p_drive_dist.timeout = 300;
    p_drive_dist.max_voltage = 2.5;
    chassis.drive_distance(30,p_drive_dist);
    wait(0.3,sec);
    
    chassis.drive_to_point(30.662,46.399);
    p_turn_point = {};
    p_turn_point.timeout = 200;
    chassis.turn_to_point(57.2,46.4,p_turn_point);
    p_drive_dist = {};
    p_drive_dist.timeout = 1500;
    p_drive_dist.max_voltage = 4;
    chassis.drive_distance(-30,p_drive_dist);
    assembly.outtake_motor.spin(forward,12,volt);

    return "";
}

std::string red_left_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    
    return "";
}
std::string red_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string red_left_elim(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }
    
    return "";
}
std::string red_right_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }

    return "";
}
std::string red_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "red right sawp, 2 long goal + mid"; }
    if (calibrate) {
        chassis.set_coordinates(-39.709, -6.756, 90);
        
        return "";
    }

    odom_constants();
    
    chassis.drive_to_point(-46.422,-47.413);

    // line up for loader and go to
    turn_to_point_params p_turn_point = {};
    p_turn_point.timeout = 300;
    chassis.turn_to_point(-55,-47.413,p_turn_point);
    assembly.tongue_piston.toggle();
    wait(0.1,sec);
    chassis.drive_to_point(-55,-47.413); 
    
    //drive into the loader
    drive_distance_params p_drive_dist = {};
    p_drive_dist.timeout = 1000;
    p_drive_dist.max_voltage = 4;
    chassis.drive_distance(10, p_drive_dist);
    wait(1,sec);

    //drive to and then into long goal
    chassis.drive_to_point(-29.246,-47.413); // add params (heading reversed)
    assembly.tongue_piston.toggle();
    assembly.intake_motor.spin(forward,12,volt);
    assembly.outtake_motor.spin(forward,12,volt);
    chassis.drive_distance(-10, p_drive_dist); // add params (short timeout and slow speed)
    wait(2,sec);
    assembly.outtake_motor.stop(hold);

    // get to first cluster
    chassis.drive_to_point(-41.972,-47.37);
    p_turn_point = {};
    p_turn_point.timeout = 500;
    chassis.turn_to_point(-24,-26.4,p_turn_point);
    chassis.drive_to_point(-24,-26.4);

    //drive to second cluster
    chassis.turn_to_point(-22,21);
    chassis.drive_to_point(-22,21);

    // line up and score mid goal
    p_turn_point = {};
    p_turn_point.angle_offset = 180;
    p_turn_point.timeout = 500;
    chassis.turn_to_point(-13.7,12.9, p_turn_point); // add heading reversed
    chassis.drive_to_point(-13.7,12.9); // add heading reversed
    assembly.mid_goal_piston.toggle();
    wait(1,sec);
    assembly.mid_goal_piston.toggle();

    // line up for loader and go to and into
    chassis.drive_to_point(-46.42,46.67);
    chassis.turn_to_point(-55,46.67);
    assembly.tongue_piston.toggle();
    wait(0.1,sec);
    chassis.drive_to_point(-55,46.67);
    chassis.drive_distance(10,p_drive_dist);
    wait(1,sec);

    // drive into loader
    //drive to and then into long goal
    chassis.drive_to_point(-29.246,46.67); // add params (heading reversed)
    assembly.tongue_piston.toggle();
    assembly.intake_motor.spin(forward,12,volt);
    assembly.outtake_motor.spin(forward,12,volt);
    chassis.drive_distance(-10, p_drive_dist); // add params (short timeout and slow speed)

    return "";
}
std::string red_right_elim(bool calibrate, auto_variation var, bool get_name) {   
    if (get_name) { return "red right elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}

std::string skills(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "skills"; }
    if (calibrate) {
        chassis.set_coordinates(-39.709, -6.756, 90);


        return "";
    }
    odom_constants();
    turn_to_point_params p_turn_point = {};
    p_turn_point.angle_offset = 0;
    p_turn_point.timeout = 500;
    chassis.turn_to_point(-45,-46);
    wait(0.1,sec);
    chassis.drive_to_point(-45,-46);
    wait(0.1,sec);
    // turn_to_point_params p_turn_point = {};
    // p_turn_point = {};
    // p_turn_point.angle_offset = 180;
    // p_turn_point.timeout = 500;
    // chassis.turn_to_point(-45,-60,p_turn_point);
    chassis.turn_to_angle(270);
    wait(0.1,sec);
    chassis.turn_to_point(-60,-45);
    wait(0.1,sec);
    chassis.drive_to_point(-60,-45);
    wait(1.5,sec);
    chassis.drive_to_point(-55,-45);
    wait(0.1,sec);
    chassis.turn_to_point(-30,-60);
    wait(1,sec);
    chassis.drive_to_point(-30,-60);
    wait(0.1,sec);
    chassis.turn_to_angle(90);
    // wait(0.1,sec);
    // chassis.drive_to_point()
    // p_turn_point = {};
    // p_turn_point.angle_offset = 90;
    // p_turn_point.timeout = 500;
    // chassis.turn_to_point(-40,-40,p_turn_point);
    // wait(0.1,sec);
    // chassis.drive_to_point(-40,-40);
    // wait(0.1,sec);
    // p_turn_point = {};
    // p_turn_point.angle_offset = 90;
    // p_turn_point.timeout = 500;
    // chassis.turn_to_point(60,-40,p_turn_point);
    // wait(0.1,sec);
    // chassis.drive_to_point(60,-40);

    


    return "";
}