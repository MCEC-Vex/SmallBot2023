#include <string>
#include <vector>
#include <math.h>
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

// ! This 2 lines helps the bot to move gradually from 0 to 100 and then 100 to 0
// * This is a bell curve function using sin function
double currentTime = static_cast<double>(pros::millis()) / 1000.0; // get time in seconds
double maxVelocity = (std::sin(currentTime) + 1) / 2 * 140; // oscillate between 0 and 100
	std::shared_ptr<ChassisController> bot = ChassisControllerBuilder()     
			.withMotors(13,12,-19,-20)  // front right and back right were reversed in order to go forward   
			// change P then D first then I only if necessary  
			//start with P I and D with zero 
			
			.withGains( //0.7, 0, 0.1 results: faster, shaking less violently 0//

				// ! DON'T CHANGE THE VALUES
				{0.003, 0.001, 0}, // Distance controller gains 
				{0.0045, 0.005,0.00008}, // turn controller gains
				{0, 0, 0}	// Angle controller (helps bot drive straight)
				)
			.withMaxVelocity(maxVelocity)
			// Green gearset, 3 inch wheel diam, 9 inch wheel track
			.withDimensions(AbstractMotor::gearset::green, {{4_in, 8.5_in}, imev5GreenTPR})
			.build();
// * This function is to turn the bot to the left
void negativeTurn(int degrees)
{
	// !Reset's the IMU's rotation 
    imu_sensor.tare_rotation();
	bot->setMaxVelocity(100);

	bot->turnAngle(-degrees*1_deg); // Turn the bot to the left

	// * Solve the greater Error - so If the error is -40 degrees, ask the bot to turn -40 degrees
	if(imu_sensor.get_rotation() >= -degrees)
	{
		int value = degrees - abs(imu_sensor.get_rotation()); // Gets The error
		pros::lcd::set_text(2, "IMU 1st Turn: " + value);
		bot->turnAngle(-round(value)*1_deg); // Turns it
	}

	// * Solve minor error, So we turned the bot -40 degrees in above examples 
	// * but bot is not accurate so it just turned -30 degrees 
	// * so until it's not greater than or equal to desired degrees, change -(error which is 10 in this case) more degrees
	while(imu_sensor.get_rotation() >= -degrees)
	{
		int value = degrees - abs(imu_sensor.get_rotation()); // Gets The error
		bot->turnAngle(-round(value)*1_deg); // Turns it
		pros::lcd::set_text(3, "IMU 2nd Turn: " + value);
		if(imu_sensor.get_rotation() <= -(degrees) || round(abs(value)) == 0) // break the loop when it's greatar than it
		{
			break;
		}
	}
}



void slapBall(){
	// ! I have to check this function again! 
	// TODO: Try this
	// double time = pros::millis() / 1000.0; // get time in seconds
    // double period = 5.0; // adjust this to change how quickly the velocity increases and decreases
    // double midpoint = period / 2;
    // double width = period / 4; // adjust this to change the width of the bell curve

    // // Calculate the velocity based on the current time
    // double maxVelocity;
    // if (time < midpoint - width) {
    //     // Increase from 50 to 100
    //     maxVelocity = 50 + (time / (midpoint - width)) * 50;
    // } else if (time < midpoint + width) {
    //     // Stay at 100
    //     maxVelocity = 100;
    // } else {
    //     // Decrease from 100 to 50
    //     maxVelocity = 100 - ((time - (midpoint + width)) / (period - (midpoint + width))) * 50;
    // }

    // // Ensure the velocity stays within the range 50-100
    // maxVelocity = std::max(50.0, std::min(100.0, maxVelocity));

	// * This function move fowared and backward to slap the ball and than turn 90 to remove all balls in fornt of it to reset the bot on wall!
		bot->moveDistance(-8_in);
		bot->moveDistance(8_in);
		bot->moveDistance(-8_in);
		bot->turnAngle(-90_deg);
		bot->turnAngle(90_deg);
}

void getBall() {
	Intake.move_velocity(-100);
	pros::delay(100);
	Intake.move_velocity(200);
}

void autonomous() {

	// * This gets the preload ball and than turn the Left Piston out to get the ball
	getBall();
	Piston_left.set_value(true); // Set piston state accordingly
	pros::delay(400);
 
	// * This function sets the bot to maxVelocity(which helps to move bot without any jerk) 
	// * and sets itself to slap balls and gets the matchload ball out of the match load
	bot->moveDistance(-14_in);
	bot->moveDistance(6_in);

	// * This calls the slapBall function to get the ball out of the matchload
	// * reset the bot to the wall
	// * and than move back to go through the aisle
	slapBall();	
	bot->moveDistance(-1_ft); 
	pros::delay(100);
	bot->moveDistance(7.5_in);

	// * This turn the bot to move through the aisle
	// * and than move forward to push the ball
	// * and than turn the Right Piston out to get the ball
	negativeTurn(88);
	bot->moveDistance(-4_ft);
	Piston_right.set_value(true); // Set piston state accordingly

	// * This move the bot to -5 deg so that the wings don't get stuck in the wall
	// * and than move ahead
	// * Sets the velocity to 110 to slap the ball out of it's way so it can get the matchload ball!
	// ! How I slap the ball - turn the bot -90 degree and than to 110 degree in force so that it can slap the ball out of it's way
	bot->turnAngle(-5_deg);
	bot->moveDistance(-2.5_ft);
	bot->setMaxVelocity(110);
	bot->turnAngle(-90_deg);
	bot->turnAngle(100_deg);

	// * This moves the bot ahead to touch the matchload bar and than turn the bot to -45 degree
	// * and turn -45 degrees to get the matchload ball [ 🤓 8 Points hahahha ]
	bot->moveDistance(-1.3_ft);
	negativeTurn(45);
	bot->setMaxVelocity(maxVelocity);
	bot->moveDistance(-1_ft);

	// * get the intake ball in first line of code out [ 🤓 2 more point ]
	// * Spank the ball hard to get out of the way
	// ! How I slap the ball - turn the bot -90 degree and than to 110 degree in force so that it can slap the ball out of it's way
	Intake.move_velocity(-150);
	bot->turnAngle(-90_deg);
	bot->turnAngle(120_deg);

	// * This moves the bot to the wall to reset and than moves back to touch the pole!
	// * Turn -30 degrees if it doesn't touch the pole
	// TODO: Test it 3 more times to make sure it touches the pole
	bot->setMaxVelocity(maxVelocity);
	bot->moveDistance(-1_ft);
	pros::delay(200);
	bot->turnAngle(3_deg);
	bot->moveDistance(8_ft);
	bot->turnAngle(-30_deg);

	// ! THANK YOU FOR READING THE CODE! 🤓
}

bool PistonROpen = true; //boolean variable to control right piston
bool PistonLOpen = true; //boolean variable to control left piston
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
