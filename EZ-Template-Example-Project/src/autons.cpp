#include "EZ-Template/drive/drive.hpp"
#include "main.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
#define HUE_BLUE_MIN 200 // Adjust as needed (lower bound of blue hue range)
#define HUE_BLUE_MAX 260 // Adjust as needed (upper bound of blue hue range)
#define DISTANCE_THRESHOLD 150
#define HUE_RED_MIN 1    // Lower bound of red hue range (main)
#define HUE_RED_MAX 30   // Upper bound of red hue range (main)
#define HUE_RED_MIN_ALT 0  // Alternate lower bound for red hue (if sensor detects it near 360)
#define HUE_RED_MAX_ALT 359 

extern pros::Rotation horizontal_encoder;
extern pros::Rotation verical_encoder;
extern pros::Motor Intake1;
extern pros::MotorGroup Intake;
extern pros::Motor Arm1;
extern pros::Motor Arm2;

extern pros::adi::DigitalOut Mogo;
extern pros::adi::DigitalOut Intake_lift;
extern pros::adi::DigitalOut Claw;
extern pros::adi::DigitalOut Claw_drop;
extern pros::adi::DigitalOut Odom_Wheel;
extern pros::Optical Color_sensor;
extern pros::Rotation Rotation_sensor;
extern pros::Distance Distance_sensor;

extern ez::PID liftPID;


void set_lift1(int input) {
  Arm1.move(input);
  Arm2.move(input);
}

void lift_auto(double target) {
 liftPID.target_set(target);
  ez::exit_output exit = ez::RUNNING;
  while (liftPID.exit_condition({Arm1, Arm2}, true) == ez::RUNNING) {
    double output = liftPID.compute(Arm1.get_position());
    set_lift1(output);
    pros::delay(ez::util::DELAY_TIME);
  }
  set_lift1(0);
}

void NoBlue(){
  bool isSorting = true;
  int sortTimer = 0;

  int color = 0;

  while (true){
double hueValue = Color_sensor.get_hue();     
if (hueValue >= HUE_BLUE_MIN && hueValue <= HUE_BLUE_MAX)   {
  color = 1;
} if (hueValue >= HUE_RED_MIN && hueValue <= HUE_RED_MAX)   {
  color = 0;
} 
    if (color == 1) {  // Change to 0 for red sorting, 1 for blue sorting
                int distance = Distance_sensor.get_distance();
                if (distance < DISTANCE_THRESHOLD) {
                    pros::delay(50);
                    Intake.move(-127);
                    pros::delay(100);
                    Intake.move(127);
                }
            }    

        }
  pros::delay(15);
}

int intake = 0;

void NoRed(){
  bool isSorting = true;
  int sortTimer = 0;
  int intake = 0;
  int color = 1;

  while (true){
double hueValue = Color_sensor.get_hue();     
if (hueValue >= HUE_BLUE_MIN && hueValue <= HUE_BLUE_MAX)   {
  color = 1;
} if (hueValue >= HUE_RED_MIN && hueValue <= HUE_RED_MAX)   {
  color = 0;
} 
    if (color == 0) {  // Change to 0 for red sorting, 1 for blue sorting
                int distance = Distance_sensor.get_distance();
                if (distance < DISTANCE_THRESHOLD) {
                    pros::delay(50);
                    Intake.move(-127);
                    pros::delay(100); 
                    Intake.move(127); // Start sorting timer
                }
            }    

        }
  pros::delay(25);
}
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(20, 0, 100);
  chassis.pid_turn_constants_set(3.6, 0.05, 28, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 300_ms, 300_ms);

  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Blue Solo Win Point For Sig
///
void drive_example() {

  chassis.drive_angle_set(-71);

  lift_auto(-1500);

  chassis.pid_drive_set(-13_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-34_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-14_in);
  chassis.pid_speed_max_set(40);
  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait_quick_chain();
  
  Mogo.set_value(true);
  lift_auto(0);


  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  Intake.move(127);

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(28_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();


  Intake.move(115);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick();




  chassis.pid_drive_set(49_in, DRIVE_SPEED, true);
  Mogo.set_value(false); 
  chassis.pid_wait_until(12_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait_quick_chain();
  Intake.move(0);

  pros::Task WeDontWantBlue(NoRed);

 

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();


  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-14_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait_quick();
  Mogo.set_value(true);

  pros::delay(120);
  chassis.pid_drive_set(1_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  Intake.move(127);


  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(35_in, 40, true);
  chassis.pid_wait_quick_chain();



 

}

///
// Blue pos
///
void turn_example() {

  pros::Task WeDontWantBlue(NoRed);
  chassis.drive_angle_set(-37);

  lift_auto(-1500);

  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-27_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-10_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait_quick();
  Mogo.set_value(true);
  lift_auto(0);
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  Intake.move(127);

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick();


  Intake.move(127);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-6_in, 120, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 120, true);
  chassis.pid_wait_quick_chain();
   chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 120, true);
  chassis.pid_wait_quick_chain(); 


//Change this bottom part for your elims auton 

///////////////////////
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(60_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait_quick();
  
///////////////////////////
  

//for elims
/*
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  Mogo.set_value(false);

  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(35_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-10_in, 50, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-10_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait_quick();
  Mogo.set_value(true);
*/
}

///
// Blue Neg
///
void drive_and_turn() {
  pros::Task WeDontWantBlue(NoRed);
  chassis.drive_angle_set(71);

  lift_auto(-1500);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-34_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-14_in);
  chassis.pid_speed_max_set(40);
  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait_quick_chain();
  
  Mogo.set_value(true);
  lift_auto(0);



  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  Intake.move(127);
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();



  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  Intake_lift.set_value(true);

  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-1_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  Intake_lift.set_value(false);
  chassis.pid_wait_quick_chain();




//Use the below for your Quals Auto for pole touch

  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(115_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(45_in, 40, true);
  chassis.pid_wait_quick_chain();



//Elims Auton here
/*
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-1_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(70_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  */
}

///
// Red Solo Winpoint Positive Starting Side
///
void wait_until_change_speed() {
  pros::Task WeDontWantBlue(NoBlue);
  chassis.drive_angle_set(37);

  lift_auto(-1500);

  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-27_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-10_in);
  chassis.pid_speed_max_set(40);
  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait_quick();
  Mogo.set_value(true);
  lift_auto(0);
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  Intake.move(127);

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  WeDontWantBlue.remove();
  Intake.move(105);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  Mogo.set_value(false); 

  chassis.pid_drive_set(15_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(38_in, 75, true);
  chassis.pid_wait_quick_chain();
  Intake.move(0);

  chassis.pid_drive_set(-2_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
 

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();


  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-14_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait_quick();
  Mogo.set_value(true);
  pros::Task Yummers(NoRed);
  pros::delay(120);
  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  Intake.move(127);


  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(35_in, 40, true);
  chassis.pid_wait_quick_chain();
}

///
// Red Positive
///
void swing_example() {


}

///
// Red Negative
///
void motion_chaining() {



}

///
// Skills
///
void combining_movements() {
  pros::Task WeDontWantBlue(NoBlue);
  Intake.move(127);
  pros::delay(200);

  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-13_in);
  chassis.pid_speed_max_set(40);
  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait_quick_chain();

  Mogo.set_value(true);
  pros::delay(100);

  chassis.pid_drive_set(1_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(25_in, 120, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, 40);
  chassis.pid_wait_quick_chain();

  lift_auto(-315);

  chassis.pid_drive_set(13_in, 80, true);
  chassis.pid_wait_quick_chain();

  pros::delay(800);
  Intake.move(0);

  chassis.pid_drive_set(-1_in, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  pros::delay(100);
  lift_auto(-1700);

  Intake.move(127);
  lift_auto(0);

  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  lift_auto(0);

  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  Intake.move(127);


//65


  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-13_in, 40, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, 70);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  Intake.move(127);
  pros::delay(300);

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  pros::delay(400);
  Intake.move(-127);
  pros::delay(100);
  Mogo.set_value(false);
  pros::delay(300);
  Intake.move(0);

  chassis.pid_drive_set(14_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-45_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-30_in, 40, true);
  chassis.pid_wait_quick_chain();

  Mogo.set_value(true);//grab second mogo
  Intake.move(127);
  pros::delay(200);
  
  chassis.pid_drive_set(1_in, 40, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(25_in, 120, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, 40);
  chassis.pid_wait_quick_chain();

  lift_auto(-315);

  chassis.pid_drive_set(13_in, 80, true);
  chassis.pid_wait_quick_chain();

  pros::delay(700);

  chassis.pid_drive_set(-2_in, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(70_deg, 40);
  chassis.pid_wait_quick_chain();
  Intake.move(0);



  pros::delay(100);
  lift_auto(-1800);

  Intake.move(127);
  lift_auto(0);

  chassis.pid_turn_set(90_deg, 40);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-13_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  lift_auto(0);

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  Intake.move(100);
  chassis.pid_drive_set(65_in, 50, true);
  chassis.pid_wait_until(30_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-13_in, 40, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, 50);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  Intake.move(127);
  pros::delay(300);

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  Intake.move(-127);
  pros::delay(300);
  Mogo.set_value(false);

  pros::delay(100);
  Intake.move(127);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  Intake.move(0);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  Intake.move(127);
  pros::delay(300);
  Intake.move(0);

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(45_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-45_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  Intake_lift.set_value(true);

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  Intake.move(127);
  chassis.pid_drive_set(75_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(75_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();



  





}

///
// Random Tug Thingy
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// Blue Solo Winpoint Negative
void interfered_example() {
 
  chassis.pid_drive_set(-28_in, 65, true);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -135_deg, SWING_SPEED, 15);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

}

// . . .
// Make your own autonomous functions here!
// . . .