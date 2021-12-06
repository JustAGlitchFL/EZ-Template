/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <list>

#include "main.h"

using namespace ez;

// Constructor for integrated encoders
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio)
    : imu(imu_port),
      left_tracker(-1, -1, false),   // Default value
      right_tracker(-1, -1, false),  // Default value
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = false;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch caluclation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

// Constructor for tracking wheels plugged into the brain
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports)
    : imu(imu_port),
      left_tracker(abs(left_tracker_ports[0]), abs(left_tracker_ports[1]), util::is_reversed(left_tracker_ports[0])),
      right_tracker(abs(right_tracker_ports[0]), abs(right_tracker_ports[1]), util::is_reversed(right_tracker_ports[0])),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = true;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch caluclation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

// Constructor for tracking wheels plugged into a 3 wire expander
Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
             int imu_port, double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port)
    : imu(imu_port),
      left_tracker({expander_smart_port, abs(left_tracker_ports[0]), abs(left_tracker_ports[1])}, util::is_reversed(left_tracker_ports[0])),
      right_tracker({expander_smart_port, abs(right_tracker_ports[0]), abs(right_tracker_ports[1])}, util::is_reversed(right_tracker_ports[0])),
      ez_auto([this] { this->ez_auto_task(); }) {
  is_tracker = true;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch caluclation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

void Drive::set_defaults() {
  // PID Constants
  headingPID = {11, 0, 20, 0};
  forward_drivePID = {0.45, 0, 5, 0};
  backward_drivePID = {0.45, 0, 5, 0};
  turnPID = {5, 0.003, 35, 15};
  swingPID = {7, 0, 45, 0};
  leftPID = {0.45, 0, 5, 0};
  rightPID = {0.45, 0, 5, 0};

  // Slew constants
  set_slew_min_power(80, 80);
  set_slew_distance(7, 7);

  // Exit condition constants
  set_exit_condition(turn_exit, 100, 3, 500, 7, 500, 500);
  set_exit_condition(swing_exit, 100, 3, 500, 7, 500, 500);
  set_exit_condition(drive_exit, 80, 50, 300, 150, 500, 500);

  // Modify joystick curve on controller (defaults to disabled)
  toggle_modify_curve_with_controller(true);

  // Left / Right modify buttons
  set_left_curve_buttons(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);
}

double Drive::get_tick_per_inch() {
  CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

  if (is_tracker)
    TICK_PER_REV = CARTRIDGE * RATIO;
  else
    TICK_PER_REV = (50.0 * (3600.0 / CARTRIDGE)) * RATIO;  // with no cart, the encoder reads 50 counts per rotation

  TICK_PER_INCH = (TICK_PER_REV / CIRCUMFERENCE);
  return TICK_PER_INCH;
}

void Drive::set_pid_constants(PID pid, double p, double i, double d, double p_start_i) {
  pid.set_constants(p, i, d, p_start_i);
}

void Drive::set_tank(int left, int right) {
  if (pros::millis() < 1500) return;

  for (auto i : left_motors) {
    i.move_voltage(left * (12000.0 / 127.0));
  }
  for (auto i : right_motors) {
    i.move_voltage(right * (12000.0 / 127.0));
  }
}

void Drive::set_drive_current_limit(int mA) {
  if (abs(mA) > 2500) {
    mA = 2500;
  }
  for (auto i : left_motors) {
    i.set_current_limit(abs(mA));
  }
  for (auto i : right_motors) {
    i.set_current_limit(abs(mA));
  }
}

// Motor telemetry
void Drive::reset_drive_sensor() {
  left_motors.front().tare_position();
  right_motors.front().tare_position();
  if (is_tracker) {
    left_tracker.reset();
    right_tracker.reset();
    return;
  }
}

int Drive::right_sensor() {
  if (is_tracker)
    return right_tracker.get_value();
  return right_motors.front().get_position();
}
int Drive::right_velocity() { return right_motors.front().get_actual_velocity(); }
double Drive::right_mA() { return right_motors.front().get_current_draw(); }
bool Drive::right_over_current() { return right_motors.front().is_over_current(); }

int Drive::left_sensor() {
  if (is_tracker)
    return left_tracker.get_value();
  return left_motors.front().get_position();
}
int Drive::left_velocity() { return left_motors.front().get_actual_velocity(); }
double Drive::left_mA() { return left_motors.front().get_current_draw(); }
bool Drive::left_over_current() { return left_motors.front().is_over_current(); }

void Drive::reset_gyro(double new_heading) { imu.set_rotation(new_heading); }
double Drive::get_gyro() { return imu.get_rotation(); }

bool Drive::imu_calibrate() {
  imu.reset();
  int time = pros::millis();
  int iter = 0;
  int delay = 10;
  while (imu.get_status() & pros::c::E_IMU_STATUS_CALIBRATING) {
    iter += delay;

    if (iter > 2990) {
      printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
      return false;
    }
    pros::delay(delay);
  }
  master.rumble(".");
  printf("IMU is done calibrating (took %d ms)\n", iter);
  return true;
}

// Brake modes
void Drive::set_drive_brake(pros::motor_brake_mode_e_t brake_type) {
  for (auto i : left_motors) {
    i.set_brake_mode(brake_type);
  }
  for (auto i : right_motors) {
    i.set_brake_mode(brake_type);
  }
}

void Drive::initialize() {
  init_curve_sd();

  imu_calibrate();
}
