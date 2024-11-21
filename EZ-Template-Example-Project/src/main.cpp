#include "main.h"




#define Intake1_port -4  //main
#define Intake2_port 
#define Arm1_port -6
#define Arm2_port 16

//Pnuematics
#define Mogo_port 8
#define Intake_lift_port 1
#define Claw_port 5
#define Claw_drop_port 9
#define Odom_Wheel_port 4

//sensors
#define Optical_port 17
#define Rotation_port 7
#define Limit_port 3
#define Distance_port 14

        pros::Controller controller (pros::E_CONTROLLER_MASTER);
        pros::Motor Intake1 (Intake1_port);
        pros::Motor Intake2 (Intake2_port);
        pros::MotorGroup  Intake ({Intake1_port , Intake2_port});
        pros::Motor Arm1 (Arm1_port);
        pros::Motor Arm2 (Arm2_port);
        
        //Pnuematics
        pros::adi::DigitalOut Mogo (Mogo_port);
        bool MogoS = 0;
        pros::adi::DigitalOut Intake_lift (Intake_lift_port);
        bool Intake_liftS = 0;
        pros::adi::DigitalOut Claw (Claw_port);
        bool ClawS = 0;
        pros::adi::DigitalOut Claw_drop (Claw_drop_port);
        bool Claw_dropS = 0;
        pros::adi::DigitalOut Odom_Wheel (Odom_Wheel_port);
        bool OdomS = 0;

        //sensors
        pros::Optical Color_sensor (Optical_port);
        pros::Rotation Armangle (Rotation_port);
        pros::adi::Button Limit_sensor (Limit_port);
        pros::Distance Distance_sensor(Distance_port);

ez::PID liftPID{0.5, 0, 1.5, 0, "Lift"};

void set_lift(int input) {
  Arm1.move(input);
  Arm2.move(input);
}


// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {11, -12, -13},     // Left Chassis Ports (negative port will reverse it!)
    {-1, 2, 3},  // Right Chassis Ports (negative port will reverse it!)

    15,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450); 
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
 Arm1.tare_position();
  //Arm2.tare_position();
  Armangle.reset();
  liftPID.exit_condition_set(80, 50, 300, 150, 500, 500);
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  //chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("Blue Solo AWP \n\n+ Blue Solo Win Point Start on Pos + ", drive_example),
      Auton("Blue Pos Side\n\n+ Setup on Blue Pos Side +", turn_example),
      Auton("Blue Neg\n\nBlue NEg Side Setup", drive_and_turn),
      Auton("Red Solos Win \n\nRed Solo WinPoint Pos Starting Side", wait_until_change_speed),
      Auton("Red Positive\n\nRed Positive Starting Side ", swing_example),
      Auton("Red Negative\n\nRed Negative Starting Side", motion_chaining),
      Auton("SKILLS SKills yipeee", combining_movements),
      Auton("Blue Solo winpoint Negative Side ---\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  Color_sensor.set_led_pwm(100);
  Arm1.tare_position();
  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}



 

#define HUE_BLUE_MIN 200 // Adjust as needed (lower bound of blue hue range)
#define HUE_BLUE_MAX 260 // Adjust as needed (upper bound of blue hue range)
#define DISTANCE_THRESHOLD 80
#define HUE_RED_MIN 0    // Lower bound of red hue range (main)
#define HUE_RED_MAX 30   // Upper bound of red hue range (main)
#define HUE_RED_MIN_ALT 330  // Alternate lower bound for red hue (if sensor detects it near 360)
#define HUE_RED_MAX_ALT 360 




void opcontrol() {
  // This is preference to what you like to drive on


  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  bool isSorting = false;
  int sortingStartTime = 0;
  int intake = 0;

  //Arm1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //Arm2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  Color_sensor.set_led_pwm(100);

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

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_B)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
  Claw_dropS = !Claw_dropS; Claw_drop.set_value(Claw_dropS ? 1 : 0);}
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
  ClawS = !ClawS; Claw.set_value(ClawS ? 1 : 0);}
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
  MogoS = !MogoS; Mogo.set_value(MogoS ? 1 : 0);}
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
  Intake_liftS = !Intake_liftS; Intake_lift.set_value(Intake_liftS ? 1 : 0);} 







if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
  Intake.move(127);
  intake = 1;}  
else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
  Intake.move(-127);
  intake = 2;} else {
    Intake.move(0);
  }


if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
  liftPID.target_set(-320);
  isSorting = false;}
else if (master.get_digital(DIGITAL_DOWN)) {
  liftPID.target_set(100);
  isSorting = true;}
else if (master.get_digital(DIGITAL_L2)){
  liftPID.target_set(-1500);
  isSorting = true;}
  set_lift(liftPID.compute(Arm1.get_position()));

    chassis.opcontrol_arcade_flipped(ez::SPLIT);  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}