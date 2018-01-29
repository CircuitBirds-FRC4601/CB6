#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <WPILib.h>
#include <CameraServer.h>
#include <unistd.h>
#include <IterativeRobot.h>
#include <opencv2/core/core.hpp>
#include <Encoder.h>


class Robot: public frc::IterativeRobot {
public:
	float lDrive,rDrive;
	float climby;

	Spark *fLeft =new Spark(0);
	Spark *bLeft =new Spark(1);
	Spark *fRight =new Spark(2);
	Spark *bRight =new Spark(3);
	Spark *gantry =new Spark(4);

	frc::Encoder *flimflam =new Encoder(0,1);
	cs::UsbCamera cam= CameraServer::GetInstance()->StartAutomaticCapture();

	Joystick *leftStick =new Joystick(0);
	Joystick *rightStick =new Joystick(1);
	Joystick *gamePad =new Joystick(2);





	frc::RobotDrive *robotDrive =new frc::RobotDrive (fLeft,bLeft,fRight,bRight);

	void RobotInit() {

		cam.SetBrightness(1200);
		cam.SetExposureManual(42);
		cam.SetWhiteBalanceManual(3800);

	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	void AutonomousInit() override {

	}
	//AUTO END


	void TeleopInt() {
		// integrating time-to-forget
	}
	//TELE START
	void TeleopPeriodic() {

		lDrive=1*leftStick->GetRawAxis(1);
		rDrive=1*rightStick->GetRawAxis(1);
		robotDrive->TankDrive(lDrive,rDrive);
		climby = gamePad->GetRawAxis(1);

		if (fabs(climby) < .1) {
			climby = 0;
		}
		gantry->Set(climby);

		SmartDashboard::PutNumber("Output", climby );
		SmartDashboard::PutNumber("Raw", gamePad->GetRawAxis(1));
		SmartDashboard::PutNumber("WHEEEEEEEEELLLLSS", flimflam->Get());
	}
};

START_ROBOT_CLASS(Robot)

/* Hardware map of the robot "TBA"  (CB5)
 *	1ft=883.95 Wheel Encoders
 *
 *		RRio Pins
 * 		PWM
 *		0 Front Left
 *		1 Back	"
 *		2 Front	Right
 *		3 Back	"
 *		4 Gantry
 *		5
 *		6
 *		7
 *		8
 *		9
 *
 *
 *  	DIO
 *  	0 Left Encoder A
 *  	1 	  "		   B
 *  	2 Right Encoder A
 *  	3	   "		B
 *  	4
 *  	5
 *  	6
 *  	7
 *  	8
 *  	9
 *  	10
 *
 *
 *  	Analog
 *  	0
 *  	1
 *  	2
 *  	3
 *
 *		Relay
 *		0
 *		1
 *		2
 *		3
 *
 *
 *
 *
 *
 *
 */
