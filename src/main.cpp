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
    // arm.move_absolute(450 * TICKS_PER_ARM_DEGREE, 90);
    // pros::delay(2000);

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
    pros::delay(2000);
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

   
    // // Move the arm with position control
    arm.move_absolute(450 * TICKS_PER_ARM_DEGREE, 90);
    pros::delay(2000);


    pros::delay(500);

    
    arm.move(127);
    pros::delay(1000);
    arm.move(0);

    chassis.moveToPoint(12, -26.1, 4000, {.forwards = false, .minSpeed = 115});

    // clamp.set_value(true);
    clamp.set_value(true);
    pros::delay(1500);
    clamp.set_value(false);
    

    intake.move(127);
    hook.move(127);

    chassis.moveToPose(3.25, -43.9, 134.6, 4000, {.minSpeed = 70});

    chassis.moveToPose(10, -58, 225, 4000, {.minSpeed = 70});
    chassis.moveToPose(11.8, -58, 203 , 4000, {.minSpeed = 70});
    
    
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

    
    
// }

void autonomous() {
    // skills();
    // redleft();
    blueLeft(); // also red right 
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





