#include "main.h"
#include <cmath>
#include "pros/misc.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "lemlib/api.hpp"


// Global objects
pros::Controller master(pros::E_CONTROLLER_MASTER);
// pros::MotorGroup left_motors({-5, -8, -10}, pros::MotorGearset::blue);
// pros::MotorGroup right_motors({14, 3, 18}, pros::MotorGearset::blue);
pros::v5::Rotation armRotation(1);
pros::Motor bottom(-2, pros::v5::MotorGears::green);
pros::Motor top(20, pros::v5::MotorGears::green);
pros::v5::Optical optical(9);
pros::Motor arm(19, pros::v5::MotorGears::green);
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut intakeRaise('B');
pros::adi::DigitalOut clamp('A');
pros::adi::DigitalOut ejector('D');
pros::adi::DigitalOut rightDoinker('E');

pros::Rotation horizontalEncoder(13);
pros::Rotation verticalEncoder(12);
pros::IMU imu(7);
pros::Distance distance(6);


pros::MotorGroup left_motors({-5, -8, -10}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({14, 3, 18}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	10.5, // 10 inch track width
	lemlib::Omniwheel::NEW_325, // using new 4" omnis
	450, // drivetrain rpm is 450
	2 // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEncoder, lemlib::Omniwheel::NEW_2, 3);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0.5);

lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, 
    &horizontal_tracking_wheel,
    nullptr,
    // vertical tracking wheel 2, set to nullptr as we are using IMEs
	&imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
											0, // integral gain (kI)
											24, // derivative gain (kD)
											3, // anti windup
											1, // small error range, in inches
											100, // small error range timeout, in milliseconds
											2, // large error range, in inches
											500, // large error range timeout, in milliseconds
											30 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
											0, // integral gain (kI)
											15, // derivative gain (kD)
											3, // anti windup
											1, // small error range, in degrees
											100, // small error range timeout, in milliseconds
											3, // large error range, in degrees
											500, // large error range timeout, in milliseconds
											0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
					lateral_controller, // lateral PID settings
					angular_controller, // angular PID settings
					sensors // odometry sensors
);

void printBrain(){
    while(true){
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %.2f", chassis.getPose().x);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %.2f", chassis.getPose().y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %.2f", chassis.getPose().theta);
        // pros::screen::print(pros::E_TEXT_MEDIUM, 3, "%i", armRotation.get_position());
        // pros::screen::print(pros::E_TEXT_MEDIUM, 4, "VERT VAL: %i", verticalEncoder.get_position());
        // pros::screen::print(pros::E_TEXT_MEDIUM, 5, "HORI VAL: %i", horizontalEncoder.get_position());
        // pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Heading: %.2f", imu.get_heading());
        // pros::screen::print(pros::E_TEXT_MEDIUM, 7, "HUE: %.2f", optical.get_hue());
        // pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Distance: %i", distance.get_distance());
        pros::delay(200);
    }
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double kp = 0.02;
double ki = 0;
double kd = 0;
double start_i = 2000;

double output = 0;
double cur;
double error = 0;
double target = 2800;
double loadedTarget = 9000;
double prev_error = 0;
double integral = 0;
double derivative = 0;

double armPID(double current){
	error = target - current;
    derivative = error - prev_error;
    
    if (ki != 0) {
        if (fabs(error) < start_i){
            integral += error;
        }
        if (sgn(error) != sgn(prev_error)){
            integral = 0;
        }
    }

    output = (error * kp) + (integral * ki) + (derivative * kd);

    prev_error = error;

  return output;
}

double armPIDAuto(double current, bool hold, bool score){
    if(score){
        error = 16500 - current;
    }
    else if(hold){
        error = 9000 - current;
    }
    else{
        error = target - current;
    }

    derivative = error - prev_error;
    
    if (ki != 0) {
        if (fabs(error) < start_i){
            integral += error;
        }
        if (sgn(error) != sgn(prev_error)){
            integral = 0;
        }
    }

    output = (error * kp) + (integral * ki) + (derivative * kd);

    prev_error = error;

  return output;
}

bool spinTop = false;
bool spinBottom = false;
bool reverse = false;
bool blue = false;
bool red = false;

bool auton = true;
bool telop = true;

bool holdRing = false;
bool scoreStop = false;

bool prime = false;
int countAuton = 0;

bool moveIntake = false;


void intakeAuton(){
    while (auton) {
        if(moveIntake){
            while(-180 > fmod(top.get_position(), 960)){
                top.move_velocity(-200);
            }
            top.move_velocity(0);
            moveIntake = false;
        }
        else if(holdRing){
            bottom.move_velocity(200);
            top.move_velocity(200); 
            pros::delay(120);

            if (top.get_actual_velocity() < 20) {
                top.move_velocity(-200);
                pros::delay(120);
                top.move_velocity(200);
            }

            if(red && optical.get_hue() > 0 && optical.get_hue() < 10 && optical.get_proximity() == 255){
                top.move_velocity(0);
                bottom.move_velocity(200);
                holdRing = false;
            }
            else if(blue && optical.get_hue() > 220 && optical.get_hue() < 230 && optical.get_proximity() == 255){
                top.move_velocity(0);
                bottom.move_velocity(200);
                holdRing = false;
            }
            countAuton = 0;
        }
        else if(reverse){
            bottom.move_velocity(0);
            top.move_velocity(-200);
            countAuton = 0;
        }
        else if (spinTop && spinBottom) {
            bottom.move_velocity(600);
            top.move_velocity(600);
            if(blue && optical.get_hue() > 0 && optical.get_hue() < 10 && optical.get_proximity() == 255){ // eject red
                top.move_velocity(200);

                ejector.set_value(true);
                pros::delay(300);
                top.move_velocity(0);
                pros::delay(200);
                ejector.set_value(false);
            }
            else if(red && optical.get_hue() > 220 && optical.get_hue() < 230 && optical.get_proximity() == 255){// eject blue
                top.move_velocity(200);

                ejector.set_value(true);
                pros::delay(300);
                top.move_velocity(0);
                pros::delay(200);
                ejector.set_value(false);
            }

            pros::delay(120);
            if(prime && distance.get_distance() < 300 && countAuton < 2 && top.get_actual_velocity() < 20){
                top.move_velocity(-200);
                pros::delay(100);
                top.move_velocity(200);
                countAuton++;
                pros::delay(300);
            }
            else if (top.get_actual_velocity() < 20 && countAuton < 2){
                top.move_velocity(-200);
                pros::delay(120);
                top.move_velocity(200);
            }
        } else if (spinBottom && !spinTop && !scoreStop) {
            top.move_velocity(0);
            bottom.move_velocity(200);
            countAuton = 0;
        } else if(spinBottom && !spinTop && scoreStop){
            top.move_velocity(-70);
            bottom.move_velocity(200);
            countAuton = 0;
        }else if (!spinBottom && spinTop){
            top.move_velocity(200);
            bottom.move_velocity(0);
            countAuton = 0;
        } else{
            top.move_velocity(0);
            bottom.move_velocity(0);
            countAuton = 0;
        }
        pros::delay(5);
    }
}

bool hold = false;
bool score = false;
bool rushHover = false;
void armAuton(){
    while(auton){
        if(score){
            double thing = armPIDAuto(armRotation.get_position(), false, true);
            arm.move_velocity(thing);
        }
        else if(prime){
            double thing = armPIDAuto(armRotation.get_position(), false, false);
            arm.move_velocity(thing);
        }
        else if(hold){
            double thing = armPIDAuto(armRotation.get_position(), true, false);
            arm.move_velocity(thing);
        }

        pros::delay(20);
    }
}

int count = 0;
bool armToggle = false;

void intakeTelop() {
    while (telop) {
        if(blue && optical.get_hue() > 0 && optical.get_hue() < 15 && optical.get_proximity() == 255){
            top.move_velocity(200);

            ejector.set_value(true);
            pros::delay(260);
            top.move_velocity(0);
            pros::delay(200);
            ejector.set_value(false);
        }
        else if(red && optical.get_hue() > 210 && optical.get_hue() < 240 && optical.get_proximity() == 255){// blue
            top.move_velocity(200);

            ejector.set_value(true);
            pros::delay(260);
            top.move_velocity(0);
            pros::delay(200);
            ejector.set_value(false);
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            bottom.move_velocity(-200);
            top.move_velocity(-200);
            count = 0;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            bottom.move_velocity(200);
            top.move_velocity(200);

            pros::delay(120);
            if(armToggle && distance.get_distance() > 100 && distance.get_distance() < 300 && count < 1 && top.get_actual_velocity() < 20){
                top.move_velocity(-200);
                pros::delay(100);
                top.move_velocity(200);
                count++;
                pros::delay(300);
            }
            else if (top.get_actual_velocity() < 20 && count < 1) {
                top.move_velocity(-200);
                pros::delay(120);
                top.move_velocity(200);
            }
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            count = 0;
            top.move_velocity(0);
            bottom.move_velocity(200);
        } else {
            count = 0;
            top.move_velocity(0);
            bottom.move_velocity(0);
        }
        pros::delay(5);
    }
}

void armTelop(){
    while(telop){
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            armToggle = true;
        }

        if(armToggle){
            double thing = armPID(armRotation.get_position());
            // if(thing != 0){
                arm.move_velocity(thing);
            // 
            // else{
            //     arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            //     arm.move_velocity(0);
            // }
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            arm.move_velocity(200);
            armToggle = false;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            arm.move_velocity(-200);
            armToggle = false;
        }else if (!armToggle) {
            arm.move_velocity(0);
        }
        pros::delay(20);
    }
}

//__________________________________________________________________________________________________________________
// pros::Task intakeAutonControl(intakeAuton);
// pros::Task armAutonControl(armAuton);
// pros::Task armTelopControl(armTelop);
// pros::Task intakeTelopControl(intakeTelop);

void initialize(){
    chassis.setPose(0, 0, -57.8);
    pros::Task updateScreen(printBrain);
    imu.reset();
    chassis.calibrate();
    armRotation.reset_position();
    optical.set_led_pwm(100);
    top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void opcontrol() {
    auton = false;
    telop = true;
    pros::Task armTelopControl(armTelop);
    pros::Task intakeTelopControl(intakeTelop);
    // armRotation.set_position(target + 300); // for skills
    // blue = true; ////////////////////////////////////////////////////////////////////////////////////////

    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    bool intakeToggle = false;
    bool clampToggle = false;
    bool doinkerToggle = false;
    bool intakeTopToggle = false;
    bool rightDoinkerToggle = false;
    bool ejectorToggle = false;
    top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            clampToggle = !clampToggle;
            clamp.set_value(clampToggle);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            rightDoinkerToggle = !rightDoinkerToggle;
            rightDoinker.set_value(rightDoinkerToggle);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            ejectorToggle = !ejectorToggle;
            ejector.set_value(ejectorToggle);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            doinkerToggle = !doinkerToggle;
            doinker.set_value(doinkerToggle);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intakeToggle = !intakeToggle;
            intakeRaise.set_value(intakeToggle);
        }

        int LeftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int RightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.9;

        left_motors.move_velocity((LeftY + RightX) * 4.7244);
        right_motors.move_velocity((LeftY - RightX) * 4.7244);

        pros::delay(20);
    }
}


//____________________________________________________________________________________________________________________

// void skills(){
//     red = true;
//     blue = false;
//     chassis.setPose(0, 0, 180);

//     arm.move_velocity(200);
//     pros::delay(800);
//     arm.move_velocity(0);
    
//     chassis.moveToPose(0, 6, 180, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
    
//     arm.move_velocity(-200);
    
//     chassis.turnToHeading(90, 1500, {.maxSpeed = 50, .minSpeed = 30});
//     chassis.moveToPoint(-17, 10,2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 50}, false);
//     arm.move_velocity(0);
//     clamp.set_value(true);
//     pros::delay(200);

//     chassis.turnToHeading(0, 1000);

//     spinTop = true;
//     spinBottom = true;
//     chassis.moveToPose(-20, 33, 0, 2000, {.maxSpeed = 50, .minSpeed = 30});
//     chassis.turnToHeading(-45, 1000, {.maxSpeed = 50, .minSpeed = 30}, false);
//     chassis.moveToPose(-43, 95, 0, 2500, {.maxSpeed = 70, .minSpeed = 30});

//     pros::delay(1500);
//     hold = false;
//     prime = true;

//     chassis.moveToPoint(-38, 65, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);

//     chassis.turnToHeading(-90, 1000, {.maxSpeed = 50, .minSpeed = 30}, false);
//     chassis.moveToPoint(-70, 65, 1500, {.maxSpeed = 50, .minSpeed = 30}, false);
//     chassis.setPose(-65, 65, chassis.getPose().theta);
//     pros::delay(100);
//     prime = false;
//     scoreStop = true;
//     spinTop = false;

//     score = true;
//     pros::delay(1000);
//     scoreStop = false;

//     chassis.moveToPoint(-55, 65, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
//     score = false;
//     hold = true;
//     spinTop = true;
//     chassis.moveToPoint(-55, 65, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);

//     chassis.turnToHeading(-180, 1000, {}, false);
//     chassis.moveToPoint(-55, -1, 2500, {.maxSpeed = 50}, false);
//     chassis.turnToHeading(-50, 1000);
//     chassis.moveToPoint(-65, 15, 1000, {.minSpeed = 40}, false);
//     chassis.turnToHeading(20, 1000);
//     chassis.moveToPoint(-75, -20, 700, {.forwards = false, .minSpeed = 40}, false);

//     reverse = true;
//     clamp.set_value(false);
//     pros::delay(200);


//     chassis.setPose(-51, 8, chassis.getPose().theta); // remove 180
//     pros::delay(200);
//     chassis.moveToPoint(-26, 11, 1000, {.maxSpeed = 70, .minSpeed = 50});
//     chassis.turnToHeading(-90, 1000, {.maxSpeed = 70, .minSpeed = 50});

//     chassis.moveToPoint(20, 11,1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);

//     chassis.moveToPoint(31, 11,1000, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);
//     clamp.set_value(true);
//     pros::delay(200);
//     chassis.turnToHeading(0, 1000, {.maxSpeed = 70, .minSpeed = 50});

//     reverse = false;

//     spinTop = true;
//     spinBottom = true;
    
//     chassis.moveToPose(30, 33, 0, 1500, {.maxSpeed = 70, .minSpeed = 50});
//     chassis.turnToHeading(45, 1000, {.maxSpeed = 70, .minSpeed = 50}, false);
//     chassis.moveToPose(55, 93, 0, 2000, {.maxSpeed = 70, .minSpeed = 50});
//     pros::delay(1500);
//     hold = false;
//     prime = true;

//     chassis.moveToPoint(50, 66, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);

//     chassis.turnToHeading(90, 1000, {}, false);
//     chassis.moveToPoint(70, 66, 2000, {.maxSpeed = 50}, false);

//     prime = false;
//     scoreStop = true;
//     spinTop = false;

//     score = true;
//     pros::delay(1000);
//     scoreStop = false;

//     chassis.moveToPoint(55, 66, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);
//     // chassis.moveToPoint(52, 68, 1000, {}, false);
//     score = false;
//     hold = true;
//     spinTop = true;

//     // chassis.moveToPoint(45, 65, 500);

//     chassis.turnToHeading(90, 1000, {}, false);
//     chassis.moveToPoint(53, -20, 2500, {.maxSpeed = 60, .minSpeed = 40}, false);
//     chassis.turnToHeading(50, 1000);
//     chassis.moveToPoint(70, 12, 1500, {.minSpeed = 40}, false);
//     chassis.turnToHeading(-10, 1000);
//     chassis.moveToPoint(100, -30, 700, {.forwards = false, .minSpeed = 40}, false);

//     reverse = true;
//     pros::delay(100);
//     clamp.set_value(false);
//     pros::delay(200);

//     //____

//     chassis.setPose(91, -15, chassis.getPose().theta);
//     hold = true;
//     spinBottom = true;
//     spinTop = false;
//     chassis.moveToPoint(70, 45, 1000);
//     reverse = false;
//     holdRing = true;
//     chassis.moveToPoint(42, 55, 1500);

//     chassis.moveToPoint(38, 90, 1000);
//     chassis.turnToHeading(90, 1000);
//     intakeRaise.set_value(true);
//     chassis.moveToPoint(83, 100, 2500, {.maxSpeed = 60, .minSpeed = 50});

//     chassis.moveToPoint(18, 81, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30}, false);
//     clamp.set_value(true);
//     holdRing = false;
//     pros::delay(200);
//     spinTop = true;
//     spinBottom = true;

//     intakeRaise.set_value(false);

//     chassis.moveToPoint(80, 87, 3500, {.maxSpeed = 80, .minSpeed = 50}, false);

//     chassis.moveToPose(40, 75, -40, 3000, {.forwards = false}, false);

//     clamp.set_value(false);
//     intakeRaise.set_value(true);

//     spinBottom = false;
//     spinTop = false;
//     chassis.moveToPoint(-34, 110, 2000, {.maxSpeed = 80, .minSpeed = 70}, false);
//     hold = false;

//     chassis.moveToPoint(91, -15, 5000, {.forwards = false, .maxSpeed = 60, .minSpeed = 60});

//     pros::delay(500);
//     arm.move_velocity(200);
//     pros::delay(500);
//     arm.move_velocity(0);
//     intakeRaise.set_value(false);
//     while(imu.get_roll() > -13 && imu.get_roll() < 13){
//         pros::delay(20);
//     }

//     pros::delay(300);
//     chassis.cancelAllMotions();

//     left_motors.move_velocity(50);
//     right_motors.move_velocity(50);
//     pros::delay(1000);
//     arm.move_velocity(-200);
//     left_motors.move_velocity(0);
//     right_motors.move_velocity(0);
//     pros::delay(1500);
// }

void skills(){
    red = true;
    blue = false;
    chassis.setPose(0, 0, 180);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);
    
    chassis.moveToPose(0, 6, 180, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
    
    arm.move_velocity(-200);
    
    chassis.turnToHeading(90, 1500, {.maxSpeed = 50, .minSpeed = 30});
    chassis.moveToPoint(-17, 10,2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 50}, false);
    arm.move_velocity(0);
    clamp.set_value(true);
    pros::delay(200);

    chassis.turnToHeading(0, 1000);

    spinTop = true;
    spinBottom = true;
    chassis.moveToPose(-20, 33, 0, 2000, {.maxSpeed = 50, .minSpeed = 30});
    chassis.turnToHeading(-45, 1000, {.maxSpeed = 50, .minSpeed = 30}, false);
    chassis.moveToPose(-43, 95, 0, 2500, {.maxSpeed = 70, .minSpeed = 30});

    pros::delay(1500);
    hold = false;
    prime = true;

    chassis.moveToPoint(-38, 65, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);

    chassis.turnToHeading(-90, 1000, {.maxSpeed = 50, .minSpeed = 30}, false);
    chassis.moveToPoint(-70, 65, 1500, {.maxSpeed = 50, .minSpeed = 30}, false);
    chassis.setPose(-64, 65, chassis.getPose().theta);
    pros::delay(100);
    prime = false;
    scoreStop = true;
    spinTop = false;

    score = true;
    pros::delay(1000);
    scoreStop = false;

    chassis.moveToPoint(-55, 65, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
    score = false;
    hold = true;
    spinTop = true;
    chassis.moveToPoint(-55, 65, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);

    chassis.turnToHeading(-180, 1000, {}, false);
    chassis.moveToPoint(-55, -1, 2500, {.maxSpeed = 50}, false);
    chassis.turnToHeading(-50, 1000);
    chassis.moveToPoint(-65, 15, 1000, {.minSpeed = 40}, false);
    chassis.turnToHeading(20, 1000);
    chassis.moveToPoint(-75, -20, 700, {.forwards = false, .minSpeed = 40}, false);

    reverse = true;
    clamp.set_value(false);
    pros::delay(200);


    chassis.setPose(-52, 10, chassis.getPose().theta); // remove 180
    pros::delay(200);
    chassis.moveToPoint(-26, 11, 1000, {.maxSpeed = 70, .minSpeed = 50});
    chassis.turnToHeading(-90, 1000, {.maxSpeed = 70, .minSpeed = 50});

    chassis.moveToPoint(20, 11,1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);

    chassis.moveToPoint(31, 11,1000, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);
    clamp.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(0, 1000, {.maxSpeed = 70, .minSpeed = 50});

    reverse = false;

    spinTop = true;
    spinBottom = true;
    
    chassis.moveToPose(30, 33, 0, 1500, {.maxSpeed = 70, .minSpeed = 50});
    chassis.turnToHeading(45, 1000, {.maxSpeed = 70, .minSpeed = 50}, false);
    chassis.moveToPose(55, 93, 0, 2000, {.maxSpeed = 70, .minSpeed = 50});
    pros::delay(1500);
    hold = false;
    prime = true;

    chassis.moveToPoint(50, 66, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.moveToPoint(70, 66, 2000, {.maxSpeed = 50}, false);

    prime = false;
    scoreStop = true;
    spinTop = false;

    score = true;
    pros::delay(1000);
    scoreStop = false;

    chassis.moveToPoint(55, 66, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30}, false);
    // chassis.moveToPoint(52, 68, 1000, {}, false);
    score = false;
    hold = true;
    spinTop = true;

    // chassis.moveToPoint(45, 65, 500);

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.moveToPoint(53, -20, 2500, {.maxSpeed = 60, .minSpeed = 40}, false);
    chassis.turnToHeading(50, 1000);
    chassis.moveToPoint(70, 12, 1500, {.minSpeed = 40}, false);
    chassis.turnToHeading(-10, 1000);
    chassis.moveToPoint(100, -30, 700, {.forwards = false, .minSpeed = 40}, false);

    reverse = true;
    pros::delay(100);
    clamp.set_value(false);
    pros::delay(200);

    //____

    chassis.setPose(91, -15, chassis.getPose().theta);
    hold = true;
    spinBottom = true;
    spinTop = false;
    chassis.moveToPoint(70, 45, 1000);
    reverse = false;
    holdRing = true;
    chassis.moveToPoint(42, 55, 1500);

    chassis.moveToPoint(38, 90, 1000);
    chassis.turnToHeading(90, 1000);
    intakeRaise.set_value(true);
    chassis.moveToPoint(83, 100, 2500, {.maxSpeed = 60, .minSpeed = 50});

    chassis.moveToPoint(18, 81, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30}, false);
    clamp.set_value(true);
    holdRing = false;
    pros::delay(200);
    spinTop = true;
    spinBottom = true;

    intakeRaise.set_value(false);

    chassis.moveToPoint(80, 87, 3500, {.maxSpeed = 80, .minSpeed = 50}, false);

    chassis.moveToPose(40, 75, -40, 3000, {.forwards = false}, false);

    clamp.set_value(false);
    intakeRaise.set_value(true);

    spinBottom = false;
    spinTop = false;
    chassis.moveToPoint(-34, 110, 2000, {.maxSpeed = 80, .minSpeed = 70}, false);
    hold = false;

    chassis.moveToPoint(91, -15, 5000, {.forwards = false, .maxSpeed = 60, .minSpeed = 60});

    pros::delay(500);
    arm.move_velocity(200);
    pros::delay(500);
    arm.move_velocity(0);
    intakeRaise.set_value(false);
    while(imu.get_roll() > -13 && imu.get_roll() < 13){
        pros::delay(20);
    }

    pros::delay(300);
    chassis.cancelAllMotions();

    left_motors.move_velocity(50);
    right_motors.move_velocity(50);
    pros::delay(1000);
    arm.move_velocity(-200);
    left_motors.move_velocity(0);
    right_motors.move_velocity(0);
    pros::delay(1500);
}

void goalSideBlue(){
    red = false;
    blue = true;
    chassis.setPose(0, 0, 180);
    chassis.turnToHeading(140, 600);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);

    chassis.moveToPose(-11, 45, 180, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    arm.move_velocity(-200);
    pros::delay(200);
    spinBottom = true;
    spinTop = true;

    chassis.turnToHeading(-90, 700, {}, false);
    arm.move_velocity(0);

    spinTop = true;
    spinBottom = true;
    chassis.moveToPoint(-30, 38, 2000, {.maxSpeed = 70});
    pros::delay(500);
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(20, -35, 2000, {.maxSpeed = 40});

    chassis.turnToHeading(-150, 500);
    chassis.moveToPoint(-50, 5, 1000, {.maxSpeed = 70, .minSpeed = 60}, false);
    chassis.moveToPoint(-70, -15, 2000, {.maxSpeed = 50, .minSpeed = 40}, false);
    chassis.moveToPoint(-40, 10, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40}, false);
    chassis.turnToHeading(45, 1000);
    chassis.moveToPoint(-15, 35, 1500, {.maxSpeed = 70}, false);
    arm.move_velocity(200);
    pros::delay(1000);
}

void ringSideBlue(){
    red = false;
    blue = true;
    pros::delay(500);

    chassis.setPose(0, 5, 180);
    chassis.turnToHeading(-140, 700, {}, false);
    arm.move_velocity(200);
    pros::delay(700);
    arm.move_velocity(0);
    chassis.moveToPose(11, 45, 180, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    arm.move_velocity(-200);
    pros::delay(200);
    chassis.turnToHeading(90, 1000);
    spinBottom = true;
    spinTop = true;
    chassis.moveToPoint(30, 38, 2500, {.maxSpeed = 90, .minSpeed = 70}, false);
    arm.move_velocity(0);
    pros::delay(200);
    chassis.turnToHeading(0, 1000);
    
    chassis.moveToPoint(38, 48, 2000, {.maxSpeed = 60, .minSpeed = 50}, false);
    chassis.moveToPoint(25, 20, 2500, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
    chassis.moveToPoint(65, -20, 2000, {.maxSpeed = 50, .minSpeed = 40}, false);
    chassis.moveToPoint(5, 30, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
    chassis.turnToHeading(-45, 1000);
    chassis.moveToPose(4, 32,  -45, 500, {.minSpeed = 50});
    arm.move_velocity(200);
    pros::delay(2000);
}

void ringSideBlueElim(){
    red = false;
    blue = true;

    chassis.setPose(0, 0, 57.8);

    arm.move_velocity(200);
    pros::delay(700);
    arm.move_velocity(0);

    chassis.moveToPoint(-8, -35, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 60}, false);
    arm.move_velocity(-200);
    clamp.set_value(true);

    pros::delay(200);
    chassis.turnToHeading(-130, 1000, {}, false);
    arm.move_velocity(0);
    spinTop = true;
    spinBottom = true;
    chassis.moveToPoint(-20, -49, 2000, {.maxSpeed = 70, .minSpeed = 50}, false);

    chassis.moveToPose(-46, -51, -90, 2000, {.maxSpeed = 70, .minSpeed = 50}, false);

    chassis.moveToPoint(-25, -49, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 69}, false);

    chassis.turnToHeading(-70, 500);

    chassis.moveToPoint(-30, -46, 1500, {.maxSpeed = 70, .minSpeed = 70}, false);

    chassis.turnToHeading(0, 1000);

    chassis.moveToPose(-45, -10, -45, 1000, {.maxSpeed = 100, .minSpeed = 100}, false);
    chassis.moveToPoint(-70, 5, 1500, {.maxSpeed = 50, .minSpeed = 50}, false);

    chassis.moveToPoint(-35, -20, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 50}, false);

    chassis.turnToHeading(90, 1000);

    chassis.moveToPoint(-22, -22, 1500, {.maxSpeed = 80, .minSpeed = 80}, false);
    
    intakeRaise.set_value(true);

    chassis.moveToPoint(-5, -20, 2000, {.maxSpeed = 40, .minSpeed = 40}, false);
    intakeRaise.set_value(false);
    chassis.moveToPoint(-10, -20, 1000, {.forwards = false});
    chassis.moveToPoint(-7, -20, 1000);
}

void goalRushBlue(){
    armRotation.set_position(0);
    red = false;
    blue = true;

    chassis.setPose(0, 0, 18);
    spinBottom = true;
    chassis.moveToPoint(10, 33, 1500,{.minSpeed = 127}, false);
    doinker.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(10, 20, 5000, {.forwards = false, .minSpeed = 100}, false);
    pros::delay(200);
    doinker.set_value(false);
    pros::delay(300);
    chassis.turnToHeading(-190, 1000);
    chassis.moveToPoint(3, 30, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 40}, false);
    clamp.set_value(true);
    pros::delay(200);
    spinTop = true;
    chassis.turnToHeading(0, 1000);
    pros::delay(500);
    spinTop = false;
    chassis.moveToPoint(-5, 20, 1000, {.forwards = false}, false);
    clamp.set_value(false);
    chassis.moveToPoint(0, 25, 1500, {.maxSpeed = 50, .minSpeed = 40}, false);
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(20, 27, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 40}, false);
    clamp.set_value(true);
    spinTop = true;

    pros::delay(200);

    chassis.turnToHeading(-150, 1000);
    doinker.set_value(true);
    chassis.moveToPoint(-25, -30, 2000, {.maxSpeed = 60, .minSpeed = 60}, false);
    chassis.turnToHeading(30, 2000, {.minSpeed = 70}, false);
    doinker.set_value(false);
    chassis.moveToPoint(5, 15, 1000);
    chassis.turnToHeading(-90, 1000);
}

void soloAWPBlue(){
    red = false;
    blue = true;

    chassis.setPose(0, 0, 180);
    chassis.turnToHeading(-140, 700, {}, false);
    arm.move_velocity(200);
    pros::delay(700);
    arm.move_velocity(0);
    chassis.moveToPose(12, 45, -180, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    arm.move_velocity(-200);
    pros::delay(200);
    spinBottom = true;
    spinTop = true;
    chassis.moveToPoint(30, 40, 1500, {.maxSpeed = 70, .minSpeed = 50}, false);
    arm.move_velocity(0);
    chassis.turnToHeading(-120, 500);
    chassis.moveToPose(5, 23, -90, 5000, {.maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(false);
    spinTop = false;
    holdRing = true;
    chassis.moveToPoint(-35, 23, 3000, {.maxSpeed = 50, .minSpeed = 40});
    chassis.turnToHeading(180, 1000);

    chassis.moveToPoint(-39, 42, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    pros::delay(200);

    chassis.moveToPoint(-62, 45, 1500, {.maxSpeed = 90, .minSpeed = 60}, false);
    holdRing = false;
    spinTop = true;

    chassis.moveToPoint(-37, 43, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    chassis.turnToHeading(35, 1000);
    arm.move_velocity(200);
}


void goalSideRed(){ // positive corner
    red = true;
    blue = false;
    chassis.setPose(0, 0, 180);
    chassis.turnToHeading(-140, 600);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);

    chassis.moveToPose(11, 45, 180, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    arm.move_velocity(-200);
    pros::delay(200);
    spinBottom = true;
    spinTop = true;

    chassis.turnToHeading(90, 700, {}, false);
    arm.move_velocity(0);

    spinTop = true;
    spinBottom = true;
    chassis.moveToPoint(30, 38, 2000, {.maxSpeed = 70});
    pros::delay(500);
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(20, -35, 2000, {.maxSpeed = 40});

    chassis.turnToHeading(150, 500);
    chassis.moveToPoint(50, 5, 1000, {.maxSpeed = 70, .minSpeed = 60}, false);
    chassis.moveToPoint(70, -15, 2000, {.maxSpeed = 50, .minSpeed = 40}, false);
    chassis.moveToPoint(40, 10, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40}, false);
    chassis.turnToHeading(-45, 1000);
    chassis.moveToPoint(17, 35, 1500, {.maxSpeed = 70}, false);
    arm.move_velocity(200);
    pros::delay(1000);
}

void ringSideRed(){
    red = true;
    blue = false;

    chassis.setPose(0, 0, 180);
    chassis.turnToHeading(140, 700, {}, false);
    arm.move_velocity(200);
    pros::delay(700);
    arm.move_velocity(0);
    chassis.moveToPose(-11, 45, -180, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    arm.move_velocity(-200);
    pros::delay(200);
    chassis.turnToHeading(-90, 1000);
    spinBottom = true;
    spinTop = true;
    chassis.moveToPoint(-30, 38, 2500, {.maxSpeed = 90, .minSpeed = 70}, false);
    arm.move_velocity(0);
    pros::delay(200);
    chassis.turnToHeading(0, 1000);
    
    chassis.moveToPoint(-39, 47, 2000, {.maxSpeed = 60, .minSpeed = 50}, false);
    chassis.moveToPoint(-25, 20, 2500, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
    chassis.moveToPoint(-65, -25, 2500, {.maxSpeed = 50, .minSpeed = 40}, false);
    chassis.moveToPoint(-5, 33, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 50}, false);
    chassis.turnToHeading(35, 1000);
    chassis.moveToPose(-4, 34,  45, 500, {.minSpeed = 50});
    arm.move_velocity(200);
    pros::delay(2000);
}

void ringSideRedElim(){
    red = true;
    blue = false;

    chassis.setPose(0, 0, -57.8);

    arm.move_velocity(200);
    pros::delay(700);
    arm.move_velocity(0);

    chassis.moveToPoint(11, -37, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 60}, false);
    arm.move_velocity(-200);
    clamp.set_value(true);

    pros::delay(200);
    chassis.turnToHeading(130, 1000, {}, false);
    arm.move_velocity(0);
    spinTop = true;
    spinBottom = true;
    chassis.moveToPose(22, -49, 140, 2000, {.maxSpeed = 60, .minSpeed = 60}, false);

    chassis.moveToPose(51, -51, 90, 2000, {.maxSpeed = 60, .minSpeed = 60}, false);

    chassis.moveToPoint(25, -45, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 70}, false);
    chassis.turnToHeading(80, 500);

    chassis.moveToPoint(32, -35, 1500, {.maxSpeed = 70, .minSpeed = 70}, false);

    chassis.turnToHeading(0, 1000);

    chassis.moveToPose(40, -10, 45, 1000, {.maxSpeed = 100, .minSpeed = 100}, false);
    chassis.moveToPoint(75, 10, 1200, {.maxSpeed = 50, .minSpeed = 50}, false);

    chassis.moveToPoint(35, -15, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 50}, false);

    chassis.turnToHeading(-90, 1000);

    chassis.moveToPoint(22, -20, 1500, {.maxSpeed = 80, .minSpeed = 80}, false);
    
    intakeRaise.set_value(true);

    chassis.moveToPoint(4, -18, 2000, {.maxSpeed = 40, .minSpeed = 40}, false);
    intakeRaise.set_value(false);
    chassis.moveToPoint(11, -20, 1000, {.forwards = false});
    chassis.moveToPoint(5, -20, 1000);
}


void goalRushRed(){
    armRotation.set_position(0);
    red = true;
    blue = false;

    chassis.setPose(0, 0, -18);
    spinBottom = true;
    chassis.moveToPoint(-10, 31, 1500,{.minSpeed = 127}, false);
    rightDoinker.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(-10, 20, 5000, {.forwards = false, .minSpeed = 100}, false);
    pros::delay(200);
    rightDoinker.set_value(false);
    pros::delay(500);
    chassis.turnToHeading(190, 1000);
    chassis.moveToPoint(-3, 30, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 40}, false);
    clamp.set_value(true);
    pros::delay(200);
    spinTop = true;
    chassis.turnToHeading(0, 1000);
    pros::delay(500);
    spinTop = false;
    chassis.moveToPoint(0, 20, 1000, {.forwards = false}, false);
    clamp.set_value(false);
    chassis.moveToPoint(-5, 25, 1500, {.maxSpeed = 50, .minSpeed = 40}, false);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-25, 25, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 40}, false);
    clamp.set_value(true);
    spinTop = true;
    pros::delay(200);
    chassis.turnToHeading(180, 1000, {}, false);
    chassis.moveToPoint(-15, -10, 1500, {}, false);
    chassis.moveToPoint(5, -15, 2000, {.maxSpeed = 60, .minSpeed = 60});
    pros::delay(500);
    rightDoinker.set_value(true);
    chassis.turnToHeading(0, 2000, {.minSpeed = 70}, false);
    rightDoinker.set_value(false);
    chassis.moveToPoint(-10, 15, 1000, {}, false);
    chassis.turnToHeading(90, 1000, {}, false);
    spinTop = false;
}

void goalRushRedArm(){
    red = true;
    blue = false;

    chassis.setPose(0, 0, -15.2); //changed to -20 to align with mogo (prev. -18)
    spinBottom = true;
    
    score = true;
    chassis.moveToPoint(-8.25, 32, 1500,{.minSpeed = 127}, false); // rush forward

    score = false;
    arm.move_velocity(200);
    pros::delay(400);
 
    chassis.moveToPoint(-10, 20, 5000, {.forwards = false, .minSpeed = 100}, false);
    arm.move_velocity(-200);
    pros::delay(200);

    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-35, 25, 2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 40}, false);
    clamp.set_value(true);
    spinTop = true;
    pros::delay(200);
    chassis.turnToHeading(40, 1000);
    chassis.moveToPoint(-25, 33, 1000);
    chassis.moveToPoint(-25, 5, 1500, {.forwards = false}, false);
    chassis.turnToHeading(180, 1000, {}, false);
    chassis.moveToPoint(5, -15, 2000, {.maxSpeed = 60, .minSpeed = 60});
    pros::delay(200);
    rightDoinker.set_value(true);
    chassis.turnToHeading(0, 2000, {.minSpeed = 70}, false);
    rightDoinker.set_value(false);
    chassis.moveToPoint(-10, 15, 1000, {}, false);
    chassis.turnToHeading(90, 1000, {}, false);
    spinTop = false;
}

void soloAWPRed(){
    red = true;
    blue = false;

    chassis.setPose(0, 0, 180);
    chassis.turnToHeading(140, 700, {}, false);
    arm.move_velocity(200);
    pros::delay(700);
    arm.move_velocity(0);
    chassis.moveToPose(-12, 50, -180, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    arm.move_velocity(-200);
    pros::delay(200);
    spinBottom = true;
    spinTop = true;
    chassis.moveToPoint(-30, 40, 1500, {.maxSpeed = 70, .minSpeed = 50}, false);
    arm.move_velocity(0);
    chassis.turnToHeading(120, 500);
    chassis.moveToPose(-5, 25, 90, 5000, {.maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(false);
    spinTop = false;
    holdRing = true;
    chassis.moveToPoint(35, 25, 3000, {.maxSpeed = 50, .minSpeed = 40});
    chassis.turnToHeading(180, 1000);

    chassis.moveToPoint(38, 45, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    clamp.set_value(true);
    pros::delay(200);

    chassis.moveToPoint(60, 48, 1500, {.maxSpeed = 90, .minSpeed = 60}, false);
    holdRing = false;
    spinTop = true;

    chassis.moveToPoint(37, 50, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    chassis.turnToHeading(-45, 1000);
    arm.move_velocity(200);
}

void autonomous(){
    auton = true;
    telop = false;
    // intakeAutonControl.resume();
    // armAutonControl.resume();
    // armTelopControl.suspend();
    // intakeTelopControl.suspend();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::Task intakeAutonControl(intakeAuton);
    pros::Task armAutonControl(armAuton);

    armRotation.set_position(target + 300); //add back

    top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(0, 0, 0);

    // go around obstacle to prevent hitting it
    // chassis.moveToPoint(
    //     20,
    //     20,
    //     2000
    //     // {.minSpeed=62, .earlyExitRange=10}
    //     // a minSpeed of 72 means that the chassis will slow down as
    //     // it approaches the target point, but it won't come to a full stop

    //     // an earlyExitRange of 8 means the movement will exit 8" away from
    //     // the target point
        
    // );

    // go to target position
    // chassis.moveToPose(20, 20, 0, 2000);
    // chassis.moveToPose(-11, -45, 0, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
    
    
    // chassis.moveToPose(-10, 60, -40, 5000);
    // chassis.turnToHeading(90, 10000);
    // skills();

    // goalSideBlue();
    // ringSideBlue();
    // ringSideBlueElim();
    // ringSideBlueElim5Ring();
    // goalRushBlue();
    // soloAWPBlue();
 
    goalRushRedArm();
    // ringSideRed();
    // ringSideRedElim();
    // ringSideRedElim5Ring();
    // goalRushRed();
    // soloAWPRed();
}
