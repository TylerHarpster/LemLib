#include "main.h"
#include "lemlog/logger/sinks/terminal.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "lemlib/motions/turnTo.hpp"
#include "pros/llemu.hpp"
#include "main.h"
#include "api.h" // IWYU pragma: keep
#include "lemlib/timer.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <chrono>
#include <ctime>
#include <thread>
#include <cmath>
#include "hot-cold-asset/asset.hpp"
#include "lemlib/motions/moveToPose.hpp"
#include "lemlib/motions/moveToPoint.hpp"

using namespace pros;
/*
  __  __    U  ___ u   _      ____   __   __        _       _   _    _   _        ____              _   _   
U|' \/ '|u   \/"_ \/  |"|    |  _"\  \ \ / /    U  /"\  u  |'| |'|  |'| |'|      |  _"\    ___     |'| |'|  
\| |\/| |/   | | | |U | | u /| | | |  \ V /      \/ _ \/  /| |_| |\/| |_| |\    /| | | |  |_"_|   /| |_| |\ 
 | |  | |.-,_| |_| | \| |/__U| |_| |\U_|"|_u     / ___ \  U|  _  |uU|  _  |u    U| |_| |\  | |    U|  _  |u 
 |_|  |_| \_)-\___/   |_____||____/ u  |_|      /_/   \_\  |_| |_|  |_| |_|      |____/ uU/| |\u   |_| |_|  
<<,-,,-.       \\     //  \\  |||_ .-,//|(_      \\    >>  //   \\  //   \\       |||_.-,_|___|_,-.//   \\  
 (./  \.)     (__)   (_")("_)(__)_) \_) (__)    (__)  (__)(_") ("_)(_") ("_)     (__)_)\_)-' '-(_/(_") ("_) 
*/



logger::Terminal terminal;

lemlib::MotorGroup rightMotors({11,-12,13}, 600_rpm);
lemlib::MotorGroup leftMotors({-18,19,-20}, 600_rpm);

lemlib::V5InertialSensor imu(15);

lemlib::TrackingWheel leftTracker(-5, 2.75_in, 26.5_cm);
lemlib::TrackingWheel rightTracker(4, 2.75_in, 26.5_cm);
lemlib::TrackingWheel horizontalTracker(-14, 2.75_in, -26.5_cm / 2);

lemlib::TrackingWheelOdometry odom({&imu}, {&leftTracker,&rightTracker}, {&horizontalTracker});

lemlib::PID pid(1, 1, 1);
lemlib::ExitCondition<AngleRange> exitCondition(1_stDeg, 2_sec);



pros::ADIDigitalOut wingPiston ('B');
pros::ADIDigitalOut tonguePiston ('A');
int wingState=0;
int tongueState=0;



Motor UpperIntake (-10);
Motor MiddleIntake (1,MotorGearset::green); // SET THIS HALF WATT
Motor LowerIntake (-14,MotorGearset::green); // SET THIS HALF WATT


void initialize() {
    terminal.setLoggingLevel(logger::Level::DEBUG);
    pros::lcd::initialize();

    imu.calibrate();
    pros::delay(3200);
    odom.startTask();
    pros::delay(100);
    pros::Task([&] {
        while (true) {
            auto p = odom.getPose();
            pros::lcd::print(0, "X: %f", to_in(p.x));
            pros::lcd::print(1, "Y: %f", to_in(p.y));
            pros::lcd::print(2, "Theta: %f", to_cDeg(p.orientation));
            pros::delay(10);
        }
    });
    lemlib::turnTo(90_cDeg, 100_sec, {.slew = 1},
                   {
                       .angularPID = pid,
                       .exitConditions = std::vector<lemlib::ExitCondition<AngleRange>>({exitCondition}),
                       .poseGetter = [] -> units::Pose { return odom.getPose(); },
                       .leftMotors = leftMotors,
                       .rightMotors = rightMotors,
                   });
}

void disabled() {}

// struct FollowSettings {
//         Length trackWidth = track_width;
//         std::function<units::Pose()> poseGetter = pose_getter;
//         lemlib::MotorGroup& leftMotors = left_motors;
//         lemlib::MotorGroup& rightMotors = right_motors;
// };

// void follow(const asset& path, Length lookaheadDistance, Time timeout, FollowParams params, FollowSettings settings);

void autonomous() {
	lemlib::moveToPose()
}


void opcontrol() {
	pros::Controller Controller(pros::E_CONTROLLER_MASTER);

        lemlib::turnTo(90_cDeg, 100_sec, {.slew = 1},
                   {
                       .angularPID = pid,
                       .exitConditions = std::vector<lemlib::ExitCondition<AngleRange>>({exitCondition}),
                       .poseGetter = [] -> units::Pose { return odom.getPose(); },
                       .leftMotors = leftMotors,
                       .rightMotors = rightMotors,
                   });
        
    autonomous();
	while(1){
		printf("f0o543t43t3tdit\n");
		if(Controller.get_digital_new_press(DIGITAL_Y)){wingPiston.set_value(wingState++%2);}
		if(Controller.get_digital_new_press(DIGITAL_B)){tonguePiston.set_value(tongueState++%2);}
		
		float j1=0.5*Controller.get_analog(ANALOG_RIGHT_X);
		float j3=0.5*Controller.get_analog(ANALOG_LEFT_Y);

		leftMotors.move(j3+j1);
		rightMotors.move(j3-j1);

		//storage
		if(Controller.get_digital(DIGITAL_R1)){
			LowerIntake.move(94*0.5);
			MiddleIntake.move(63*0.5);
			UpperIntake.move(63);
		}
	
		if(Controller.get_digital_new_press(DIGITAL_R1)){
			MiddleIntake.brake();
			UpperIntake.brake();
		}

		//move blocks up
		if(Controller.get_digital(DIGITAL_L1)){
			LowerIntake.move(94*0.5);
			MiddleIntake.move(127*0.5);
			UpperIntake.move(127);
		}

		//move blocks down
		if(Controller.get_digital(DIGITAL_R2)){
			LowerIntake.move(-31*0.5);
			MiddleIntake.move(-63*0.5);
			UpperIntake.move(-63);
		}

		//middle goal
		if(Controller.get_digital(DIGITAL_L2)){
			LowerIntake.move(63*0.5);
			MiddleIntake.move(94*0.5);
			UpperIntake.move(-106);
		}

		//un-middle goal
		if(Controller.get_digital(DIGITAL_X)){
			LowerIntake.move(-63*0.5);
			MiddleIntake.move(-94*0.5);
			UpperIntake.move(106);
		}

		//unstucky
		if(Controller.get_digital(DIGITAL_DOWN)){
			LowerIntake.move(-127*0.5);
			MiddleIntake.move(127*0.5);
		}

		if(Controller.get_digital(DIGITAL_RIGHT)){
			MiddleIntake.brake();
			LowerIntake.brake();
			UpperIntake.brake();
		}

		pros::delay(50);

	}


}
