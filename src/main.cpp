#include "main.h"

// Motor definitions
pros::Motor intake(6, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor kicker1(16, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor kicker2(17, pros::E_MOTOR_GEAR_200, true, pros::E_MOTOR_ENCODER_DEGREES);

// Pneumatics

// Pneumatics
ez::Piston left_wing('B');
ez::Piston right_wing('C');
ez::Piston back_wing('A');
ez::Piston hang('D');

auto leftOuterLEDs = sylib::Addrled(22, 5, 17);
auto leftInnerLEDs = sylib::Addrled(22, 6, 16);
auto rightOuterLEDs = sylib::Addrled(22, 8, 17);
auto rightInnerLEDs = sylib::Addrled(22, 7, 16);

bool code2 = false;

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

void ledInitialize() {
  leftOuterLEDs.gradient(0xFFFFFF, 0x0000ff);
  leftInnerLEDs.gradient(0xFFFFFF, 0x0000ff);
  rightOuterLEDs.gradient(0xFFFFFF, 0x0000ff);
  rightInnerLEDs.gradient(0xFFFFFF, 0x0000ff);
}

void ledCycle() {
  leftOuterLEDs.cycle(*leftOuterLEDs, 5);
  leftInnerLEDs.cycle(*leftInnerLEDs, 5);
  rightInnerLEDs.cycle(*rightInnerLEDs, 5);
  rightOuterLEDs.cycle(*rightOuterLEDs, 5);
}

void ledPulse(int hexCode) {
  leftOuterLEDs.pulse(hexCode, 2, 10);
  leftInnerLEDs.pulse(hexCode, 2, 10);
  rightOuterLEDs.pulse(hexCode, 2, 10);
  rightInnerLEDs.pulse(hexCode, 2, 10);
}

void ledKill() {
  leftOuterLEDs.set_all(0x000000);
  leftInnerLEDs.set_all(0x000000);
  rightOuterLEDs.set_all(0x000000);
  rightInnerLEDs.set_all(0x000000);
}

void setAll(int hexCode) {
  leftOuterLEDs.set_all(hexCode);
  leftInnerLEDs.set_all(hexCode);
  rightOuterLEDs.set_all(hexCode);
  rightInnerLEDs.set_all(hexCode);
}

void police(void* param) {
 while (true)
 {
  setAll(0x0000FF);
  pros::delay(50);
  leftOuterLEDs.set_all(0xFF0000);
  rightOuterLEDs.set_all(0xFF0000);
  pros::delay(10);
  setAll(0x0000FF);
  leftInnerLEDs.set_all(0xFF0000);
  rightInnerLEDs.set_all(0xFF0000);
  pros::delay(10);
 }
 
}





void initialize() {
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_drive_activebrake_set(0);
  chassis.opcontrol_curve_default_set(4, 4); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  sylib::initialize();

  ledKill();


  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    // Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    // Auton("Example Turn\n\nTurn 3 times.", turn_example),
    // Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    // Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    // Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
    // Auton("Combine all 3 movements", combining_movements),
    // Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),

    Auton("Test Auton", test_auton),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  leftOuterLEDs.set_pixel(0xFFFF00, 0);
  leftOuterLEDs.set_pixel(0xFFFF00, 16);
  rightOuterLEDs.set_pixel(0xFFFF00, 0);
  rightOuterLEDs.set_pixel(0xFFFF00, 16);
  master.rumble(".");
  ledKill();
  pros::delay(100);
  leftOuterLEDs.set_pixel(0xFFFF00, 0);
  leftOuterLEDs.set_pixel(0xFFFF00, 16);
  rightOuterLEDs.set_pixel(0xFFFF00, 0);
  rightOuterLEDs.set_pixel(0xFFFF00, 16);
  master.rumble(".");
  ledKill();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
  while (true) {
    leftOuterLEDs.set_all(0x0000FF);
    leftInnerLEDs.set_all(0x0000FF);
    rightInnerLEDs.set_all(0x0000FF);
    rightOuterLEDs.set_all(0x0000FF);
    pros::delay(200);
    leftOuterLEDs.set_all(0xffffff);
    leftInnerLEDs.set_all(0xffffff);
    rightInnerLEDs.set_all(0xffffff);
    rightOuterLEDs.set_all(0xffffff);
    pros::delay(200);
  }
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

  ledCycle();

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
  pros::Task policetask(police, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Police");
  policetask.suspend();

  ledInitialize();
  ledCycle();

  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
    //   if (master.get_digital_new_press(DIGITAL_X)) 
    //     chassis.pid_tuner_toggle();
        
    //   chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    // } 
    }

    chassis.opcontrol_arcade_standard(ez::SPLIT);

    if (master.get_digital_new_press(DIGITAL_X) && policetask.get_state() == pros::E_TASK_STATE_SUSPENDED) {
      policetask.resume();
    } else if (master.get_digital_new_press(DIGITAL_X) && policetask.get_state() == pros::E_TASK_STATE_RUNNING) {
      policetask.suspend();        
    }

  // Trigger the selected autonomous routine
    if (master.get_digital_new_press(DIGITAL_B))  {
      autonomous();
    }

    // pneumatics
    if (master.get_digital(DIGITAL_RIGHT)) {
      ledPulse(0xFFFFF);
      left_wing.set(true);
    } else {
      left_wing.set(false);
    }
    if (master.get_digital(DIGITAL_Y)) {
      ledPulse(0xFFFFF);
      right_wing.set(true);
    } else {
      right_wing.set(false);
    }
    if (master.get_digital(DIGITAL_L1)) {
      ledPulse(0xFFFFF);
      back_wing.set(true);
    } else {
      back_wing.set(false);
    }

    // Read joystick inputs for intake control
    if (master.get_digital(DIGITAL_R1)) {
        intake.move_velocity(200); 
    } else if (master.get_digital(DIGITAL_R2)) {
        intake.move_velocity(-200);
    } else {
        intake.move_velocity(0);
    }

    // Read joystick inputs for kicker control
    if (master.get_digital(DIGITAL_L2)) {
        ledPulse(0xFFFFF);
        kicker1.move_velocity(200);
        kicker2.move_velocity(200);
    } else {
        kicker1.move_velocity(0);
        kicker2.move_velocity(0);
    }

    // hang
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      kicker1.move_velocity(-200);
      kicker2.move_velocity(-200);
      hang.set(false);
    }

    if (master.get_digital_new_press(DIGITAL_A)) {
      hang.set(true);
      ledPulse(0xFFFFF);
      kicker1.move_velocity(0);
      kicker2.move_velocity(0);
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}