#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace lemlib;

pros::MotorGroup dt_left({-5, 2, -9}, pros::v5::MotorGears::blue,
                         pros::v5::MotorUnits::degrees);
pros::MotorGroup dt_right({1, -6, 19}, pros::v5::MotorGears::blue,
                          pros::v5::MotorUnits::degrees);

pros::Motor lady_brown(14, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::degrees);
pros::MotorGroup intake({11, 20}, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);
pros::Motor preroller(20, pros::v5::MotorGears::green,
                      pros::v5::MotorUnits::degrees); // intake motor on port 9
pros::Controller controller(pros::E_CONTROLLER_MASTER);
;

pros::Imu imu(8);

int intakeSpeed = 600;

pros::adi::Pneumatics clamp('A', false);

lemlib::Drivetrain drivetrain(
    &dt_left,  // left motor group
    &dt_right, // right motor group
    13,
    lemlib::Omniwheel::OLD_325, // using new 4" omnis
    480,                        // drivetrain rpm is 360
    3 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings
    linearController(6,   // proportional gain (kP) 27
                     0,   // integral gain (kI)
                     8,   // derivative gain (kD) 320
                     3,   // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     40   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(2.1, // proportional gain (kP)
                      0,   // integral gain (kI)
                      14,  // derivative gain (kD)
                      3,   // anti windup
                      1,   // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0    // maximum acceleration (slew)
    );

// Create a new rotation sensor on port 11 (adjust the port number as needed)
pros::Rotation horizontalRotation(10);

// Create a new horizontal tracking wheel using the rotation sensor .5 inches
// behind and 1 inch to the left of tracking center
lemlib::TrackingWheel horizontal1(&horizontalRotation, lemlib::Omniwheel::NEW_2,
                                  0.5);

// Update the OdomSensors object to include the new horizontal tracking wheel
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            &imu     // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttleCurve(3,    // joystick deadband out of 127
                  10,   // minimum output where drivetrain will move out of 127
                  1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               10,   // minimum output where drivetrain will move out of 127
               1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  chassis.setPose(0, 0, 0);
  lady_brown.set_zero_position_all(0);

  // the default rate is 50. however, if you need to change the rate, you
  // can do the following.
  // lemlib::bufferedStdout().setRate(...);
  // If you use bluetooth or a wired connection, you will want to have a rate of
  // 10ms

  // for more information on how the formatting for the loggers
  // works, refer to the fmtlib docs

  // thread to for brain screen and position logging
  pros::Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      pros::lcd::print(3, "LB Deg: %f", lady_brown.get_position());
      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      pros::delay(50);
    }
  });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // unclamp
  clamp.retract();
  dt_left.move_velocity(0);
  dt_right.move_velocity(0);
  intake.move_velocity(0);
  lady_brown.move_velocity(0);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void intake_score(int degrees, int direction) {
  intake.move_relative(degrees, 600 * direction);
  intake.brake();
}

void intake_on(int speed) {
  if (speed == 0) {
    intake.brake();
    return;
  }

  intake.move_velocity(600);
}

void intake_off() { intake_on(0); }

void Auton1() {
  // Autonomous winpoint blue positive side / red positive side

  // score on alliance stake

  chassis.setPose(-60, -12, 0);
  chassis.moveToPose(-60, 0, 0, 5000);
  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(-65, 0, 1000, {.forwards = false});
  intake_score(2000, 1);

  // pick up ring and score

  chassis.setPose(-62, 0, 90, false);
  intake_on(600);
  chassis.moveToPose(-24, -48, 135, 2700, {}, false);
  intake_off();

  clamp.toggle();
  chassis.turnToHeading(180, 2000);
  chassis.moveToPoint(-24, -22, 5000, {.forwards = false, .maxSpeed = 25},
                      false);
  clamp.toggle();

  pros::delay(500);
  intake_score(1000, 1);

  chassis.turnToHeading(0, 1000);
  chassis.moveToPoint(-20, -2, 5000, {.forwards = true, .maxSpeed = 40}, false);
}

void Auton2() {

  pros::delay(5000);
  // Autonomous winpoint blue negative side / red negative side

  // score on alliance stake

  chassis.setPose(-60, 24, 180);
  chassis.moveToPose(-60, 0, 180, 5000);
  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(-65, 0, 1000, {.forwards = false});
  intake_score(2000, 1);

  // pick up ring and score

  chassis.setPose(-62, 0, 90, false);
  intake_on(600);
  chassis.moveToPose(-24, 48, 45, 2700, {}, false);
  intake_off();

  clamp.toggle();
  chassis.turnToHeading(0, 2000);
  chassis.moveToPoint(-24, 22, 5000, {.forwards = false, .maxSpeed = 25},
                      false);
  clamp.toggle();

  pros::delay(500);
  intake_score(1000, 1);

  chassis.turnToHeading(180, 1000);
  chassis.moveToPoint(-20, 2, 5000, {.forwards = true}, false);
}

void Auton3() {
  chassis.setPose(0, 0, 0, false);
  chassis.moveToPose(0, -36, 0, 2700, {.forwards = false, .maxSpeed = 70},
                     false);
  clamp.extend();
  intake.move_velocity(600);
}

void Auton5() {
  // Skills challenge autonomous

  // Chassis position: coordinate from the back of the drivetrain
  // Chassis heading: front intake is direction

  // Step 1. We start under the red alliance stake.
  // With the preloaded ring, we will score on the stake using our wall stake
  // mechanism.

  chassis.setPose(-165, 0, 90, false);
  intake_score(1000, 1);
  // -- TODO: Score on the red alliance stake

  // Step 2. We will go to pick up the top left mobile goal
  // with our clamp facing into the mobile goal.

  chassis.turnToHeading(180, 5000, {}, false);

  chassis.moveToPoint(-120, 60, 5000, {.forwards = true}, false);
  pros::delay(200);

  clamp.toggle();

  // Step 3. We will go and score the 6 rings around the mobile goal onto our
  // robot. This will take a lot of precise coding and movement to nail
  // autonomously

  intake_score(1000, 1);

  // -- Score bottom right ring (1)

  chassis.turnToHeading(90, 5000, {}, false);

  chassis.moveToPoint(-60, 60, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score second top ring (2)

  chassis.turnToHeading(0, 5000, {}, false);

  chassis.moveToPoint(-60, 120, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score center top ring (3)

  chassis.turnToHeading(90, 5000, {}, false);

  chassis.moveToPoint(0, 150, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score corner center ring (4)

  chassis.turnToHeading(270, 5000, {}, false);

  chassis.moveToPoint(-120, 120, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score corner back left ring (5)

  chassis.moveToPoint(-150, 120, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score corner top ring (6)

  chassis.moveToPoint(-120, 150, 5000, {.forwards = true}, false);
  pros::delay(200);

  // Step 4. We will go and put the fully scored out mobile goal into the top
  // right corner to double its points.

  chassis.moveToPose(-168, -168, 135, 5000, {.forwards = false}, false);
  pros::delay(200);

  clamp.toggle();

  // Step 5. We will go to the center, and pick up the center ring on our robot.
  // This will later be used to score on the bottom right mobile goal.

  chassis.moveToPose(0, 0, 0, 5000, {.forwards = true}, false);
  pros::delay(500);
  intake_score(1000, 1);

  // Step 6. We will pick up the bottom right’s mobile goal to score more rings
  // onto.

  chassis.moveToPose(-120, 60, 45, 5000, {.forwards = false}, false);

  clamp.toggle();

  // Step 7. We will pick up all of the rings in the bottom right corner.
  // This will required high precision and a well-tuned autonomous to accomplish
  // quickly.

  // -- Pick up the top right ring (2)

  intake_score(1000, 1);

  chassis.moveToPose(-60, -60, 135, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up the bottom right ring (3)

  chassis.moveToPose(-60, -120, 180, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up the middle ring (4)
  chassis.moveToPose(-120, -120, 270, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up a ring (5)
  chassis.moveToPose(-150, -120, 270, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up last ring (6)
  chassis.moveToPose(-120, -150, 135, 5000, {.forwards = true}, false);
  pros::delay(200);

  // Step 8. We will put the mobile goal into the positive corner at the bottom
  // right. This will double all of the points on our current mobile goal.

  chassis.moveToPose(-166, -166, 45, 5000, {.forwards = false}, false);

  // Step 9. Move to the center line, and pick up a ring, then turn around and
  // score it on the high stakes.

  chassis.moveToPose(0, -150, 90, 5000, {.forwards = true}, false);
  // TODO: SCORE RING ON HIGH STAKE

  // Step 10. We will go to the center bar and hang
  chassis.moveToPose(-25, -40, 45, 5000, {.forwards = true}, false);
  // TODO: TOGGLE HANG MECHANISM
}

void match1() {

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(-62.4, 40.5, 270);
  chassis.moveToPoint(-45, 40.5, 1000);
  chassis.turnToPoint(-31.8, 29.4, 1000, {.forwards = false}, false);
  chassis.moveToPoint(-31.8, 29.4, 1000, {.forwards = false}, false);

  clamp.extend();

  chassis.turnToPoint(-23.3, 48.8, 1000, {}, false);
  intake.move_velocity(600);
  chassis.moveToPoint(-20.3, 50.8, 1000, {.maxSpeed = 50}, false);

  chassis.turnToPoint(-67.1, 67, 1000);
  chassis.moveToPoint(-67.1, 67, 1000);

  pros::delay(3000);

  chassis.moveToPose(-49, -23, 180, 3000);
}

void match2() {
  //red neg

  chassis.setPose(0, 0, 0, false);
  chassis.moveToPose(0, -36, 0, 2700, {.forwards = false, .maxSpeed = 70},
                     false);
  clamp.extend();
  intake.move_velocity(600);

  pros::delay(6000);

  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(0, 40, 1000);
}

void x() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 12, 800);
  chassis.turnToPoint(19, 12, 650, {.forwards = false});
  chassis.moveToPoint(16.5, 12, 850, {.forwards = false, .earlyExitRange = 2},
                      false);
  chassis.moveToPoint(19, 12, 550, {.forwards = false, .maxSpeed = 63}, false);

  // Latch the mogo - delays just in case
  clamp.extend();
  pros::delay(250);

  // Turn to the above ring - Start intake and hook, and move there
  chassis.turnToPoint(24, 36.75, 850, {}, false);
  intake.move_velocity(600);
  chassis.moveToPoint(24, 36.75, 1000, {}, false);

  chassis.turnToPoint(48, 36.75, 850, {}, false);
  chassis.moveToPoint(48, 36.75, 1000, {.maxSpeed = 40}, false);
  pros::delay(750);

  chassis.turnToPoint(45, 10, 850, {.maxSpeed = 40}, false);
  pros::delay(1000);
  chassis.moveToPoint(45, 10, 1000, {.maxSpeed = 40}, false);

  pros::delay(2000);
  chassis.moveToPoint(45, 6, 2000, {.maxSpeed = 40}, false);
  pros::delay(1000);
  chassis.turnToPoint(54.5, 6, 850, {.forwards = false, .maxSpeed = 40}, false);
  pros::delay(1000);
  chassis.moveToPoint(54.5, 1.5, 1000, {.forwards = false, .maxSpeed = 40},
                      true);
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  clamp.retract();
  intake.move_velocity(600);
  chassis.moveToPoint(-29, 13, 4000,
                      {.forwards = false, .maxSpeed = 50, .earlyExitRange = 2},
                      false);
  clamp.extend();
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  intake.move_velocity(600);
  chassis.turnToPoint(-29.8, 37, 1000, {}, false);
  pros::delay(1000);
  chassis.moveToPoint(-29.8, 37, 1000, {.maxSpeed = 50}, false);
  pros::delay(1000);
  chassis.turnToPoint(-53.8, 37, 1000, {}, false);
  pros::delay(1000);
  chassis.moveToPoint(-53.8, 37, 1000, {.maxSpeed = 50}, false);
  pros::delay(1000);
  chassis.moveToPoint(-53.8, 5, 1000, {.maxSpeed = 50}, false);
  pros::delay(1000);
  chassis.turnToPoint(-50.5, 6, 1000, {}, true);
  chassis.moveToPoint(-50.5, 6, 1000, {.maxSpeed = 50}, true);
  pros::delay(1000);
  chassis.turnToPoint(-39.5, 7, 1000, {}, true);
  chassis.moveToPoint(-39.5, 7, 1000, {.maxSpeed = 50}, false);
  chassis.turnToPoint(-67, 0, 1000, {.forwards = false}, false);
  chassis.moveToPoint(-67, 0, 1000, {.forwards = false, .maxSpeed = 50}, false);
  clamp.retract();
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  chassis.turnToPoint(-3, 62, 3000, {.forwards = true, .maxSpeed = 90}, false);
  pros::delay(1000);
  chassis.moveToPoint(-3, 62, 2000, {.forwards = true, .maxSpeed = 70}, false);
  // tune this delay
  intake.move_voltage(12000);
  pros::delay(600); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  chassis.moveToPoint(45, 104, 3000, {.forwards = true, .maxSpeed = 70}, false);
  pros::delay(1000);
  chassis.turnToPoint(-25, 108, 5000, {.forwards = false}, false);
  chassis.moveToPoint(-25, 108, 5000, {.forwards = false, .maxSpeed = 70},
                      false);
  clamp.extend();
  intake.move_velocity(600);
  chassis.moveToPoint(-64.5, 131.1, 2000, {.forwards = false}, false);
  chassis.moveToPoint(-55, 120, 500, {.forwards = true}, false);
  chassis.moveToPoint(-64.5, 131.1, 700, {.forwards = false}, false);
  clamp.retract();
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  chassis.moveToPoint(0, 120, 2000, {.forwards = true}, false);
  chassis.moveToPoint(-64.5, 131.1, 700, {.forwards = true}, false);
}

void autonomous() {
  // x();
  Auton3();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  auto start_time = std::chrono::steady_clock::now();
  bool flagged = false;
  lady_brown.move_absolute(0, 200);
  intake.move_velocity(0);
  enum LadyBrownState { IDLE, PRIMED, SCORED };

  // Static variable to track current state
  static LadyBrownState ladyBrownState = IDLE;

  // loop forever
  while (true) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                            current_time - start_time)
                            .count();

    if (elapsed_time >= 70 && !flagged) {
      controller.rumble(". - . -");
      flagged = true;
    }
    // get left y and right y positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move_velocity(intakeSpeed);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_velocity(-intakeSpeed);
    } else {
      intake.brake();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      clamp.toggle();
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      switch (ladyBrownState) {
      case IDLE:
        lady_brown.move_absolute(66, 200); // Move to primed position
        intake.move_velocity(intakeSpeed); // Start intake
        ladyBrownState = PRIMED;
        break;

      case PRIMED:
        lady_brown.move_absolute(460, 200); // Maintain primed position
        intake.move_velocity(0);            // Stop intake
        ladyBrownState = SCORED;
        break;

      case SCORED:
        lady_brown.move_absolute(0, 80); // Move to scoring position
        ladyBrownState = IDLE;
        break;
      }
    }

    pros::delay(15);
  }

  // lady brown control logic, pushing the button once will move it to primed
  // postion in degrees then run the intake motor until the button is pressed
  // again then the motor will stop then pushing the button [r1] will move the
  // lady brown to the scored position in degrees

  // delay to save resources
}
