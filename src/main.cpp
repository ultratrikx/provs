#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include <cmath>

// Motor and drivetrain declarations
pros::MotorGroup left_motors({-18, -19, -20}, pros::MotorGearset::blue); // Left motors use 600 RPM cartridges
pros::MotorGroup right_motors({1, 2, 3}, pros::MotorGearset::blue);      // Right motors use 600 RPM cartridges (assuming blue like left)
lemlib::Drivetrain drivetrain(
    &left_motors,               // Left motor group
    &right_motors,              // Right motor group
    11.25,                      // 11.25 inch track width
    lemlib::Omniwheel::NEW_275, // Using new 2.75" omnis
    450,                        // Drivetrain RPM (adjusted for blue cartridge)
    2                           // Horizontal drift is 2 (for now)
);

// IMU on port 10
pros::Imu imu(10);

// Other motors
pros::Motor intake(6);
pros::Motor hook(11);
pros::Motor arm(5);

// Pneumatics
pros::adi::DigitalOut clamp('H');
pros::adi::DigitalOut doinker('G');
pros::adi::DigitalOut pushPiston('F');

// Drive curve settings
lemlib::ExpoDriveCurve throttle_curve(
    3,     // Joystick deadband out of 127
    10,    // Minimum output where drivetrain moves out of 127
    1.019  // Expo curve gain
);

lemlib::ExpoDriveCurve steer_curve(
    3,     // Joystick deadband out of 127
    10,    // Minimum output where drivetrain moves out of 127
    1.019  // Expo curve gain
);

// Motion controllers
lemlib::ControllerSettings lateral_controller(
    10,  // kP
    0,   // kI
    3,   // kD
    3,   // Anti-windup
    1,   // Small error range (inches)
    100, // Small error timeout (ms)
    3,   // Large error range (inches)
    500, // Large error timeout (ms)
    10   // Maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
    2,   // kP
    0,   // kI
    10,  // kD
    3,   // Anti-windup
    1,   // Small error range (degrees)
    100, // Small error timeout (ms)
    3,   // Large error range (degrees)
    500, // Large error timeout (ms)
    5    // Maximum acceleration (slew)
);

// Odometry sensors
lemlib::OdomSensors sensors(
    nullptr, // Vertical tracking wheel 1
    nullptr, // Vertical tracking wheel 2
    nullptr, // Horizontal tracking wheel 1
    nullptr, // Horizontal tracking wheel 2
    &imu     // Inertial sensor
);

// Chassis
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors,
    &throttle_curve,
    &steer_curve
);

// Arm constants
const double TICKS_PER_ARM_DEGREE = 900.0 / 360.0; // 2.5 ticks per degree at arm (3:1 gear ratio)
const double DEGREES_PER_TICK = 360.0 / 900.0;     // 0.4° per tick at arm
const int MAX_ARM_ANGLE = 404;                     // Maximum allowed angle in degrees
const double MAX_ARM_TICKS = MAX_ARM_ANGLE * TICKS_PER_ARM_DEGREE; // 1010 ticks

// Global state for arm macro
bool isArmMacroRunning = false;
bool isRaised = false;
int macroTargetDegrees = 0;

// Asynchronous arm macro task
void armMacroTask(void* param) {
    while (true) {
        if (isArmMacroRunning) {
            double targetTicks = macroTargetDegrees * TICKS_PER_ARM_DEGREE;
            double armAngle = arm.get_position() * DEGREES_PER_TICK;

            if (!isRaised) {
                if (macroTargetDegrees > MAX_ARM_ANGLE) macroTargetDegrees = MAX_ARM_ANGLE;
                arm.move_absolute(targetTicks, 50); // Speed = 50
                if (fabs(macroTargetDegrees - armAngle) <= 2.0) { // ±2° tolerance
                    isRaised = true;
                    isArmMacroRunning = false;
                    arm.move_velocity(0);
                }
            } else {
                arm.move_absolute(0, 50);
                if (fabs(armAngle) <= 2.0) { // ±2° tolerance from 0°
                    isRaised = false;
                    isArmMacroRunning = false;
                    arm.move_velocity(0);
                }
            }
        }
        pros::delay(10); // Task loop delay
    }
}

/**
 * Callback for LLEMU's center button.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "working");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Initialization code.
 */
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    arm.tare_position();

    // Start the arm macro task
    pros::Task arm_task(armMacroTask, nullptr, "ArmMacroTask");

    // Screen task for position and arm angle
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
            double armAngle = arm.get_position() * DEGREES_PER_TICK;
            pros::lcd::print(3, "Arm Angle: %.1f deg", armAngle);
            pros::delay(20);
        }
    });
}

/**
 * Runs while disabled.
 */
void disabled() {}

/**
 * Runs before autonomous in competition mode.
 */
void competition_initialize() {}

/**
 * Asynchronous arm raise function.
 */
void armRaise(int targetDegrees, int speed) {
    if (!isArmMacroRunning) { // Only start if not already running
        macroTargetDegrees = targetDegrees;
        isArmMacroRunning = true; // Trigger the task
    }
}
void intakeControl(int speed, bool direction) {
    if (direction) {
        intake.move(speed);
    } else {
        intake.move(-speed);
    }
}
void hookControl(int speed, bool direction) {
    if (direction) {
        hook.move(speed);
    } else {
        hook.move(-speed);
    }
}

/**
 * Autonomous mode.
 */
void skills() {
    clamp.set_value(true);
    
    // Set arm brake mode to hold at the start
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.tare_position();
    
    // Start position: 0, 0, -40
    chassis.setPose(0, 0, -40);  // Move to (0, 0) with heading -40

    // Move the arm with position control
    arm.move_absolute(400 * TICKS_PER_ARM_DEGREE, 90);
    pros::delay(2000);

    // Move to position 11.6, -15.5, -40 (arm down position)
    chassis.moveToPoint(12, -16, 2000, {.forwards = false, .minSpeed = 78});
    
    // Clamp the mobile goal
    clamp.set_value(true);
    pros::delay(500);
    clamp.set_value(false);

    // Lower arm with position control
    arm.move_absolute(0, 90);
    pros::delay(1000);
    // Ensure arm stays at position 0 after movement
    arm.move_velocity(0);
    
    // Turn to 11.6, -15.5, -172 (align to the next target)
    chassis.moveToPose(11.6, -15.5, -172, 2000, {.minSpeed = 125});
    pros::delay(500);
    
    // Move to "Red 1" position (7.3, -29.2, -172)
    chassis.moveToPose(7.3, -29.2, -172, 2000, {.minSpeed = 125});
    intake.move(127);
    hook.move(127); 
    pros::delay(500);
    
    // Move to "Red 2" position (19.4, -84.6, -201)
    chassis.moveToPose(22, -90, -201, 2000, {.minSpeed = 125});
    intake.move(127);
    hook.move(127);
    pros::delay(500);
    
    // Get ready for high stakes (back): Move to (11, -59, -201)
    chassis.moveToPose(11, -66, -201, 2000, {.forwards = false});
    pros::delay(500);
    
    // Turn to stake position: (11, -59, -257.4)
    chassis.moveToPose(11, -66, -257.4, 2000, {.minSpeed = 125});
    
    // Use position control for consistent arm height
    arm.move_absolute(70 * TICKS_PER_ARM_DEGREE, 50);
    pros::delay(500);

    // Move forward to position (26.7, -62.2, -257.4)
    chassis.moveToPose(26.7, -66, -257.4, 2000, {.minSpeed = 125});
    
    // Move to wall position: (36.6, -64.6, -257.4)
    chassis.moveToPose(40, -66, -257.4, 2000, {.minSpeed = 125});
    pros::delay(3000);
    intake.move(0);
    hook.move(0);
    
    // Activate pushPiston while maintaining arm position
    pushPiston.set_value(true);  
    pros::delay(1500);
    intake.move(0);
    hook.move(0);
    
    // Raise arm for  stake and ensure it maintains position
    arm.move_absolute(400 * TICKS_PER_ARM_DEGREE, 50);
    pros::delay(2000);

    // Jitter forward and back (for stabilization)
    chassis.moveToPoint(40, -66, 500, {.maxSpeed = 70});
    chassis.moveToPoint(42, -66, 500, {.maxSpeed = 70});
    chassis.moveToPoint(40, -66, 500, {.maxSpeed = 70});
    chassis.moveToPoint(42, -66, 500, {.maxSpeed = 70});
    
    // Final position: (23.8, -64.6, -257.4)
    chassis.moveToPose(23.8, -66, -257.4, 3000, {.forwards = false});
    pros::delay(500);
    
    // Ensure arm stays in position at end of routine
    arm.move_absolute(0, 90);
}

// void leftRed(){
//     clamp.set_value(true);
    
//     // Set arm brake mode to hold at the start
//     arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     arm.tare_position();
    
//     // Start position: 0, 0, -40
//     chassis.setPose(0, 0, 0);  // Move to (0, 0) with heading -40

//     // Move the arm with position control
//     arm.move_absolute(400 * TICKS_PER_ARM_DEGREE, 90);
//     pros::delay(2000);

//     // Move to position 11.6, -15.5, -40 (arm down position)
//     chassis.moveToPoint(-17.1, -22.2, 2000, {.forwards = false, .minSpeed = 78});
    
//     // Clamp the mobile goal
//     clamp.set_value(true);
//     pros::delay(500);
//     clamp.set_value(false);

//     // Lower arm with position control
//     arm.move_absolute(0, 90);
//     pros::delay(1000);

//     // Ensure arm stays at position 0 after movement
//     arm.move_velocity(0);

//     intake.move(127);
//     hook.move(127);

//     arm.move_absolute(70 * TICKS_PER_ARM_DEGREE, 90);

//     chassis.moveToPose(-13, -37, -145, 3000, {.minSpeed = 70});

//     pushPiston.set_value(true);
//     pros::delay(500);

//     intake.move(0);
//     hook.move(0);


//     //score wall stake
//     chassis.moveToPose(-11.6, -64.0, 191.9,5000);
//     pros::delay(500);

//     pushPiston.set_value(false);
//     pros::delay(500);
//     arm.move_absolute(400*TICKS_PER_ARM_DEGREE, 90);



//     chassis.moveToPose(-13, -37, -145, 3000, {.forwards = false, .minSpeed = 70});

//     pros::delay(500);

//     arm.move_absolute(0, 90);  

//     intake.move(127);
//     hook.move(127);

    
//     chassis.moveToPose(9.9, -27.10, -65.7, 4000, {.minSpeed = 70});

//     chassis.moveToPose(20, -29.7, -81.30, 4000);

//     clamp.set_value(true);
//     pros::delay(500);

//     chassis.moveToPoint(28.7, -31.72, 4000, {.forwards = false, .minSpeed = 115});

//     arm.move_absolute(120, 90);
//     chassis.moveToPose(-33.75, -15.55, -84.96, 2000);


 void blueLeft(){
    clamp.set_value(true);
    
    // Set arm brake mode to hold at the start
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.tare_position();
    
    // Start position: 0, 0, -40
    chassis.setPose(0, 0, 0);  // Move to (0, 0) with heading -40

    // // Move to position 11.6, -15.5, -40 (arm down position)
    // chassis.moveToPoint(-41, -38.8, 2000, {.forwards = false, .minSpeed = 78});
    
    // // Clamp the mobile goal
    // clamp.set_value(true);
    // pros::delay(500);
    // clamp.set_value(false);


    // // Move the arm with position control
    arm.move_absolute(450 * TICKS_PER_ARM_DEGREE, 90);
    pros::delay(2000);

    // // Move to position 11.6, -15.5, -40 (arm down position)
    // chassis.moveToPoint(17.1, -27.2, 2000, {.forwards = false, .minSpeed = 78});
    
    // // Clamp the mobile goal
    // clamp.set_value(true);
    // pros::delay(500);
    // clamp.set_value(false);

    // // Lower arm with position control
    // arm.move_absolute(0, 90);
    // pros::delay(1000);

    // // Ensure arm stays at position 0 after movement
    // arm.move_velocity(0);

    // intake.move(127);
    // hook.move(127);

    // //arm.move_absolute(70 * TICKS_PER_ARM_DEGREE, 90);

    // chassis.moveToPose(9.4, -46, -155.6, 3000, {.minSpeed = 70});

    // pushPiston.set_value(true);
    // pros::delay(500);

    // intake.move(0);
    // hook.move(0);


    // //score wall stake
    // chassis.moveToPose(-11.6, -64.0, 191.9,5000);
    // pros::delay(500);

    // pushPiston.set_value(false);
    // pros::delay(500);
    // arm.move_absolute(400*TICKS_PER_ARM_DEGREE, 90);



    // chassis.moveToPose(-13, -37, -145, 3000, {.forwards = false, .minSpeed = 70});

    // pros::delay(500);

    // arm.move_absolute(0, 90);  

    // intake.move(127);
    // hook.move(127);

    
    // chassis.moveToPose(23.2, -57.8, -225.94, 4000, {.minSpeed = 70});

    // chassis.moveToPose(20, -29.7, -81.30, 4000);

    pros::delay(500);

    chassis.moveToPoint(0, -34.3, 4000, {.forwards = false, .minSpeed = 115});

    // clamp.set_value(true);
    clamp.set_value(true);
    pros::delay(1500);
    clamp.set_value(false);

    intake.move(127);
    hook.move(127);

    chassis.moveToPose(19.1, -34.3, 92, 4000, {.minSpeed = 70});

    // arm.move_absolute(120, 90);
    // chassis.moveToPose(-33.75, -15.55, -84.96, 2000);


    
} 



 void redleft(){
    clamp.set_value(true);
    
    // Set arm brake mode to hold at the start
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.tare_position();
    
    // Start position: 0, 0, -40
    chassis.setPose(0, 0, 0);  // Move to (0, 0) with heading -40

   
    // // Move the arm with position control
    arm.move_absolute(450 * TICKS_PER_ARM_DEGREE, 90);
    pros::delay(2000);


    pros::delay(500);

    
    arm.move(127);
    pros::delay(1000);
    arm.move(0);

    chassis.moveToPoint(-12, -26.1, 4000, {.forwards = false, .minSpeed = 115});

    // clamp.set_value(true);
    clamp.set_value(true);
    pros::delay(1500);
    clamp.set_value(false);
    

    intake.move(127);
    hook.move(127);

    chassis.moveToPose(-3.25, -43.9, 134.6, 4000, {.minSpeed = 70});

    chassis.moveToPose(-10, -58, 225, 4000, {.minSpeed = 70});
    chassis.moveToPose(-11.8, -58, 203 , 4000, {.minSpeed = 70});
    
} 


 void blueRight(){
    clamp.set_value(true);
    
    // Set arm brake mode to hold at the start
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.tare_position();
    
    // Start position: 0, 0, -40
    chassis.setPose(0, 0, 0);  // Move to (0, 0) with heading -40

    // // Move to position 11.6, -15.5, -40 (arm down position)
    // chassis.moveToPoint(-41, -38.8, 2000, {.forwards = false, .minSpeed = 78});
    
    // // Clamp the mobile goal
    // clamp.set_value(true);
    // pros::delay(500);
    // clamp.set_value(false);


    // // Move the arm with position control
    arm.move_absolute(450 * TICKS_PER_ARM_DEGREE, 90);
    pros::delay(2000);

    // // Move to position 11.6, -15.5, -40 (arm down position)
    // chassis.moveToPoint(17.1, -27.2, 2000, {.forwards = false, .minSpeed = 78});
    
    // // Clamp the mobile goal
    // clamp.set_value(true);
    // pros::delay(500);
    // clamp.set_value(false);

    // // Lower arm with position control
    // arm.move_absolute(0, 90);
    // pros::delay(1000);

    // // Ensure arm stays at position 0 after movement
    // arm.move_velocity(0);

    // intake.move(127);
    // hook.move(127);

    // //arm.move_absolute(70 * TICKS_PER_ARM_DEGREE, 90);

    // chassis.moveToPose(9.4, -46, -155.6, 3000, {.minSpeed = 70});

    // pushPiston.set_value(true);
    // pros::delay(500);

    // intake.move(0);
    // hook.move(0);


    // //score wall stake
    // chassis.moveToPose(-11.6, -64.0, 191.9,5000);
    // pros::delay(500);

    // pushPiston.set_value(false);
    // pros::delay(500);
    // arm.move_absolute(400*TICKS_PER_ARM_DEGREE, 90);



    // chassis.moveToPose(-13, -37, -145, 3000, {.forwards = false, .minSpeed = 70});

    // pros::delay(500);

    // arm.move_absolute(0, 90);  

    // intake.move(127);
    // hook.move(127);

    
    // chassis.moveToPose(23.2, -57.8, -225.94, 4000, {.minSpeed = 70});

    // chassis.moveToPose(20, -29.7, -81.30, 4000);

    pros::delay(500);

    chassis.moveToPoint(-7, -34.3, 4000, {.forwards = false, .minSpeed = 115});

    // clamp.set_value(true);
    clamp.set_value(true);
    pros::delay(1500);
    clamp.set_value(false);

    intake.move(127);
    hook.move(127);

    chassis.moveToPose(-23, -34.3, 92, 4000, {.minSpeed = 70});

    chassis.moveToPose(-20, -40, 92, 4000, {.minSpeed = 70});
    chassis.moveToPose(-17, -40, 92, 4000, {.minSpeed = 70});

    // arm.move_absolute(120, 90);
    // chassis.moveToPose(-33.75, -15.55, -84.96, 2000);
    
} 


void redRight(){
    clamp.set_value(true);
    
    // Set arm brake mode to hold at the start
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.tare_position();
    
    // Start position: 0, 0, -40
    chassis.setPose(0, 0, 0);  // Move to (0, 0) with heading -40

    // // Move to position 11.6, -15.5, -40 (arm down position)
    // chassis.moveToPoint(-41, -38.8, 2000, {.forwards = false, .minSpeed = 78});
    
    // // Clamp the mobile goal
    // clamp.set_value(true);
    // pros::delay(500);
    // clamp.set_value(false);


    // // Move the arm with position control
    arm.move_absolute(450 * TICKS_PER_ARM_DEGREE, 90);
    pros::delay(2000);

    // // Move to position 11.6, -15.5, -40 (arm down position)
    // chassis.moveToPoint(17.1, -27.2, 2000, {.forwards = false, .minSpeed = 78});
    
    // // Clamp the mobile goal
    // clamp.set_value(true);
    // pros::delay(500);
    // clamp.set_value(false);

    // // Lower arm with position control
    // arm.move_absolute(0, 90);
    // pros::delay(1000);

    // // Ensure arm stays at position 0 after movement
    // arm.move_velocity(0);

    // intake.move(127);
    // hook.move(127);

    // //arm.move_absolute(70 * TICKS_PER_ARM_DEGREE, 90);

    // chassis.moveToPose(9.4, -46, -155.6, 3000, {.minSpeed = 70});

    // pushPiston.set_value(true);
    // pros::delay(500);

    // intake.move(0);
    // hook.move(0);


    // //score wall stake
    // chassis.moveToPose(-11.6, -64.0, 191.9,5000);
    // pros::delay(500);

    // pushPiston.set_value(false);
    // pros::delay(500);
    // arm.move_absolute(400*TICKS_PER_ARM_DEGREE, 90);



    // chassis.moveToPose(-13, -37, -145, 3000, {.forwards = false, .minSpeed = 70});

    // pros::delay(500);

    // arm.move_absolute(0, 90);  

    // intake.move(127);
    // hook.move(127);

    
    // chassis.moveToPose(23.2, -57.8, -225.94, 4000, {.minSpeed = 70});

    // chassis.moveToPose(20, -29.7, -81.30, 4000);

    pros::delay(500);

    chassis.moveToPoint(-7, -34.3, 4000, {.forwards = false, .minSpeed = 115});

    // clamp.set_value(true);
    clamp.set_value(true);
    pros::delay(1500);
    clamp.set_value(false);

    intake.move(127);
    hook.move(127);

    chassis.moveToPose(-23, -34.3, 92, 4000, {.minSpeed = 70});

    // arm.move_absolute(120, 90);
    // chassis.moveToPose(-33.75, -15.55, -84.96, 2000);
    
} 





    
    
// }

void autonomous() {
    // skills();
    redleft();
    // blueLeft(); 
    //blueRight();
}

/**
 * Operator control mode.
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // Emergency reset
    arm.move(0);
    arm.tare_position();
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.set_voltage_limit(12000);

    pros::delay(500);

    controller.clear();
    controller.print(0, 0, "Ready");
    controller.rumble(".");

    while (true) {
        // Drive control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.curvature(leftY, rightX);

        // Manual arm control with angle limit
        double armAngle = arm.get_position() * DEGREES_PER_TICK;
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (armAngle < MAX_ARM_ANGLE) {
                arm.move_velocity(200); // Move up if below 404°
                isArmMacroRunning = false; // Cancel macro if manual used
            } else {
                arm.move_velocity(0);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            arm.move_velocity(-70); // Allow downward movement
            isArmMacroRunning = false; // Cancel macro if manual used
        } else if (!isArmMacroRunning) {
            arm.move_velocity(0); // Stop unless macro is running
        }

        // Arm Macro SET (async)
        static bool lastY = false;
        bool currentY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        if (currentY && !lastY) { // Rising edge
            armRaise(90, 50); // Trigger async macro to 95°
            controller.rumble(".");
        }
        lastY = currentY;

        // Arm Macro REST (async)
        static bool lastDOWN = false;
        bool currentDOWN = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        if (currentDOWN && !lastDOWN) { // Rising edge
            armRaise(0, 200); // Trigger async macro to 95°
            // controller.rumble(".");
        }
        lastDOWN = currentDOWN;


        // INTAKE CONTROL
        // static bool intakeRunning = false;
        // static bool intakeDirection = true; // true = forward, false = reverse
        
        
        
        // //INAKE/HOOK TOGGLE CONTROL
        // // Intake control with toggle functionality     
        // static bool lastR1 = false;
        // bool currentR1 = controller.get_digital(DIGITAL_R1);
        // if (currentR1 && !lastR1) {
        //     if (!intakeRunning) {
        //     // Start intake in forward direction
        //     intakeRunning = true;
        //     intakeDirection = true;
        //     } else if (intakeRunning && intakeDirection) {
        //     // Stop intake if it was already running forward
        //     intakeRunning = false;
        //     } else {
        //     // Change direction to forward if it was running backward
        //     intakeDirection = true;
        //     }
        // }
        // lastR1 = currentR1;
        
        // static bool lastR2 = false;
        // bool currentR2 = controller.get_digital(DIGITAL_R2);
        // if (currentR2 && !lastR2) {
        //     if (!intakeRunning) {
        //     // Start intake in reverse direction
        //     intakeRunning = true;
        //     intakeDirection = false;
        //     } else if (intakeRunning && !intakeDirection) {
        //     // Stop intake if it was already running backward
        //     intakeRunning = false;
        //     } else {
        //     // Change direction to backward if it was running forward
        //     intakeDirection = false;
        //     }
        // }
        // lastR2 = currentR2;
        
        // // Apply motor movements based on state
        // if (intakeRunning) {
        //     if (intakeDirection) {
        //     // Forward
        //     intakeControl(127, true);
        //     hookControl(127, true);
        //     } else {
        //     // Reverse
        //     intakeControl(127, false);
        //     hookControl(127, false);
        //     }
        // } else {
        //     // Stopped
        //     intakeControl(0, true);
        //     hookControl(0, true);
        // }




        // NORMAL INTAKE/HOOK CONTROL
       // Intake control
      // Intake control
        if (controller.get_digital(DIGITAL_R1)) {
        intake.move(127); hook.move(127);
        } else if (controller.get_digital(DIGITAL_R2)) {
        hook.move(-127); intake.move(-127);
        } else {
        hook.move(0); intake.move(0);
        }




        // INTAKE SOLO CONTROL
        if (controller.get_digital(DIGITAL_B)) {
            // When B button is pressed, run intake forward only 
            // Override other intake controls`
            intake.move(127);
            hook.move(0); // Stop hook
        }
        // } else if (!controller.get_digital(DIGITAL_B)) {
        //     // Only stop the intake if it's not being controlled by the toggle system
        //     // and B is not pressed
        //     intake.move(0);
        // }




        
        // Pneumatic controls
        static bool clamp_state = false;
        static bool clamp_last_a_state = false;
        bool clamp_current_a_state = controller.get_digital(DIGITAL_A);
        if (clamp_current_a_state && !clamp_last_a_state) {
            clamp_state = !clamp_state;
        }
        clamp_last_a_state = clamp_current_a_state;
        clamp.set_value(clamp_state);

        static bool doinker_state = false;
        static bool doinker_last_x_state = false;
        bool doinker_current_x_state = controller.get_digital(DIGITAL_X);
        if (doinker_current_x_state && !doinker_last_x_state) {
            doinker_state = !doinker_state;
        }
        doinker_last_x_state = doinker_current_x_state;
        doinker.set_value(doinker_state);

        static bool pushPiston_state = false;
        static bool pushPiston_last_right_state = false;
        bool pushPiston_current_right_state = controller.get_digital(DIGITAL_RIGHT);
        if (pushPiston_current_right_state && !pushPiston_last_right_state) {
            pushPiston_state = !pushPiston_state;
        }
        pushPiston_last_right_state = pushPiston_current_right_state;
        pushPiston.set_value(pushPiston_state);

        // Display position and arm values to controller
        static int displayCounter = 0;
        if (++displayCounter >= 50) { // Update every second (20ms * 50 = 1000ms)
            // Row 0: Position X and Y
            double armAngle = arm.get_position() * DEGREES_PER_TICK;

            controller.print(0, 0, "X:%.1f  Y:%.1f  T%.1f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            
            // Row 1: Heading and arm angle
            
            
            displayCounter = 0;
        }

        pros::delay(20);
    }
}


