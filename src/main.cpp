#include "main.h"

// Motor definitions
pros::Motor intake(6, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor kicker1(16, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor kicker2(17, pros::E_MOTOR_GEAR_200, true, pros::E_MOTOR_ENCODER_DEGREES);

// Pneumatics



// Chassis constructor
ez::Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is used as the sensor
  {-8, 10, -9}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is used as the sensor
  ,{18, -20, 19}

  // IMU Port
  ,5

  // Wheel Diameter
  ,3.25

  // Cartridge RPM
  ,600

  // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / MOTOR GEAR
  ,1.666
);



void initialize() {
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_default_set(4, 4); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    // Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    // Auton("Example Turn\n\nTurn 3 times.", turn_example),
    // Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    // Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    // Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
    // Auton("Combine all 3 movements", combining_movements),
    // Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),

    Auton("Close AWP", close_awp),
    Auton("Close Score", close_score),
    Auton("Far 6Ball", far_6ball),
    Auton("Far AWP", far_awp),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
void competition_initialize() {
  // . . .
}



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
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.drive_imu_reset(); // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
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
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X)) 
        chassis.pid_tuner_toggle();
        
      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 
  // Trigger the selected autonomous routine
    if (master.get_digital_new_press(DIGITAL_B)) 
      autonomous();

    chassis.opcontrol_tank(); // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
