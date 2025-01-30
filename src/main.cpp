#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

using namespace lemlib;

pros::MotorGroup dt_left({-5, 11, -16}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::MotorGroup dt_right({-4, 3, 1}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

pros::MotorGroup lady_brown({10, -21}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor intake(12);                  // intake motor on port 9
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Imu imu(2);

int intakeSpeed = 200;

pros::adi::Pneumatics clamp('H', false);

lemlib::Drivetrain drivetrain(
    &dt_left,  // left motor group
    &dt_right, // right motor group
    13,
    lemlib::Omniwheel::OLD_325, // using new 4" omnis
    480,                        // drivetrain rpm is 360
    3 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings
    linearController(10,  // proportional gain (kP)
                     0,   // integral gain (kI)
                     3,   // derivative gain (kD)
                     3,   // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     20   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(2,   // proportional gain (kP)
                      0,   // integral gain (kI)
                      10,  // derivative gain (kD)
                      3,   // anti windup
                      1,   // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0    // maximum acceleration (slew)
    );

// sensors for odometry
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
void disabled() {}

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



void intake_score(int delay, int direction) 
{  
  intake.move_velocity(600 * direction);
	pros::delay(delay);
	intake.brake();
}

void intake_on(int speed)
{
   if(speed == 0)
   {
	    intake.brake();
      return;
   }
   
   intake.move_velocity(600);
}

void intake_off()
{
  intake_on(0);
}

void Auton1()
{
   pros::delay(5000);
   //Autonomous winpoint blue positive side / red positive side

   //score on alliance stake

   chassis.setPose(-60, -12, 0);
   chassis.moveToPose(-60, 0, 0, 5000);
   chassis.turnToHeading(90, 1000);
   chassis.moveToPoint(-65, 0, 1000, {.forwards=false});
   intake_score(2000, 1);

   //pick up ring and score

   chassis.setPose(-62, 0, 90, false);
   intake_on(600);
   chassis.moveToPose(-24, -48, 135, 2700, {}, false);
   intake_off();

   clamp.toggle();
   chassis.turnToHeading(180, 2000);
   chassis.moveToPoint(-24, -22, 5000, {.forwards=false, .maxSpeed=25}, false);
   clamp.toggle();

   pros::delay(500);
   intake_score(1000, 1);

   chassis.turnToHeading(0, 1000);
   chassis.moveToPoint(-20, -2, 5000,  {.forwards=true, .maxSpeed=40}, false);
}

void Auton2()
{

   pros::delay(5000);
   //Autonomous winpoint blue negative side / red negative side

   //score on alliance stake

   chassis.setPose(-60, 24, 180);
   chassis.moveToPose(-60, 0, 180, 5000);
   chassis.turnToHeading(90, 1000);
   chassis.moveToPoint(-65, 0, 1000, {.forwards=false});
   intake_score(2000, 1);

   //pick up ring and score

   chassis.setPose(-62, 0, 90, false);
   intake_on(600);
   chassis.moveToPose(-24, 48, 45, 2700, {}, false);
   intake_off();

   clamp.toggle();
   chassis.turnToHeading(0, 2000);
   chassis.moveToPoint(-24, 22, 5000, {.forwards=false, .maxSpeed=25}, false);
   clamp.toggle();

   pros::delay(500);
   intake_score(1000, 1);

   chassis.turnToHeading(180, 1000);
   chassis.moveToPoint(-20, 2, 5000,  {.forwards=true}, false);
}

void autonomous() {
  Auton1();
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
  auto start_time = std::chrono::steady_clock::now();
  bool flagged = false;
  lady_brown.move_absolute(0, 200);
  intake.move_velocity(0);
  enum LadyBrownState { IDLE, PRIMED, SCORING, SCORED };

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
    if (
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
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
        lady_brown.move_absolute(350, 100); // Maintain primed position
        intake.move_velocity(0);           // Stop intake
        ladyBrownState = SCORING;
        break;

      case SCORING: 
        lady_brown.move_absolute(390, 75); // Move to scoring position
        ladyBrownState = SCORED;             // Reset state
        break;

      case SCORED:
        lady_brown.move_absolute(0, 200); // Move to scoring position
        ladyBrownState = IDLE;
        break;
      }
    }

    pros::delay(20);
  }

  // lady brown control logic, pushing the button once will move it to primed
  // postion in degrees then run the intake motor until the button is pressed
  // again then the motor will stop then pushing the button [r1] will move the
  // lady brown to the scored position in degrees

  // delay to save resources
}
