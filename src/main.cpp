#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include <string>
#include <vector>
#include <math.h>
#include <string>
#include "okapi/api.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
using namespace okapi;

pros::Motor FrontLeft(13, false);
pros::Motor FrontRight(12, true);
pros::Motor BackRight(19, false);
pros::Motor BackLeft(20, true);
pros::Motor Intake(15, false);
pros::ADIDigitalOut Piston_left('B');   //left wing 
pros::ADIDigitalOut Piston_right('A');  //right wing 
pros::Motor Elevation(18,false);
pros::IMU imu_sensor(16);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 *
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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
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
 * 
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	std::shared_ptr<ChassisController> bot = ChassisControllerBuilder()     
			.withMotors(-13,-12,19,20)  // front right and back right were reversed in order to go forward   
			// change P then D first then I only if necessary  
			//start with P I and D with zero 
			.withGains( //0.7, 0, 0.1 results: faster, shaking less violently 0//
				{1.0E-3, 0, 0}, // Distance controller gains 
				{0, 0, 0}, // turn controller gains
				{0, 0, 0}	// Angle controller (helps bot drive straight)
				)
			.withMaxVelocity(115)
			// Green gearset, 3 inch wheel diam, 9 inch wheel track
			.withDimensions(AbstractMotor::gearset::green, {{4_in, 8.5_in}, imev5GreenTPR})
			.build();

pros::lcd::set_text(1, "THIS IS AUTON!");
bot->moveDistance(1_ft);  
// bot->turnAngle(90_deg);
// bot->moveDistance(74_in); 
// bot->turnAngle(90_deg);
// bot->moveDistance(10_ft);
// bot->turnAngle(90_deg);
// bot->moveDistance(74_in); 


}

bool PistonROpen=true; //boolean variable to control right piston
bool PistonLOpen=true; //boolean variable to control left piston
void opcontrol() {





int yMotion;
int xMotion;
int ArmVoltage = 30;

    	pros::delay(500);

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while (true)
	{
		pros::lcd::set_text(1, "READY TO DRIVE");
		pros::lcd::set_text(2, "Front Left Motor: " + std::to_string(FrontLeft.get_position()));
		pros::lcd::set_text(3, "Front Right Motor:" + std::to_string(FrontRight.get_position()));
		pros::lcd::set_text(4, "Back Left Motor:" + std::to_string(BackLeft.get_position()));
		pros::lcd::set_text(5, "Back Right Motor:" + std::to_string(BackRight.get_position()));
		pros::lcd::set_text(7, "Intake Motor:" + std::to_string(Intake.get_position()));
		
		// driving control code

		yMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // ik this looks wrong, but it works
		xMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		

		int right = -xMotion + yMotion; //-power + turn
		int left = xMotion + yMotion;	// power + turn

		FrontLeft.move(left); // Swap negatives if you want the bot to drive in the other direction
		BackLeft.move(-left);
		BackRight.move(right);
		FrontRight.move(-right);


		if(master.get_digital(DIGITAL_R2))
			{
				Intake.move_velocity(115);

			}
		else if (master.get_digital(DIGITAL_R1))
			{
				Intake.move_velocity(-115);

			}
		else{
				Intake.move_velocity(0);
		}

		if(master.get_digital(DIGITAL_X))
		{
			Elevation.move_velocity(-120);
		}
		else{
			Elevation.move_velocity(0);
		}





		if( master.get_digital(DIGITAL_A))
		{
            PistonROpen = !PistonROpen; // Toggle piston state
            Piston_right.set_value(PistonROpen); // Set piston state accordingly	
            PistonLOpen = !PistonLOpen; // Toggle piston state
            Piston_left.set_value(PistonLOpen); // Set piston state accordingly
        	pros::delay(300);			
		}

		if(master.get_digital(DIGITAL_L2))
		{ 
            // If button A is pressed
            PistonROpen = !PistonROpen; // Toggle piston state
            Piston_right.set_value(PistonROpen); // Set piston state accordingly

        // Delay to prevent rapid checking
        pros::delay(300);
		}

		if(master.get_digital(DIGITAL_L1))
		{ 
            PistonLOpen = !PistonLOpen; // Toggle piston state
            Piston_left.set_value(PistonLOpen); // Set piston state accordingly

        // Delay to prevent rapid checking
        pros::delay(300);
		}
	}
	}