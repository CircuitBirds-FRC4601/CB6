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
private:
	frc::SendableChooser<std::string> chooser;
	 std::string autoForward= "Forward";
	const std::string autoNormal = "Let The Magic Happen";//do some Code Magic! YAH!
	const std::string autoNone = "NONE!";
	std::string autoSelected;
public:
	float lDrive,rDrive;
	float climby;
	int lDis,rDis;
	bool armout,armin;

	Spark *fLeft =new Spark(0);
	Spark *bLeft =new Spark(1);
	Spark *fRight =new Spark(2);
	Spark *bRight =new Spark(3);
	Spark *elevator =new Spark(4);

	frc::Encoder *encLeft =new Encoder(0,1);
	frc::Encoder *encRight =new Encoder(2,3);
	cs::UsbCamera cam= CameraServer::GetInstance()->StartAutomaticCapture();

	Joystick *leftStick =new Joystick(0);
	Joystick *rightStick =new Joystick(1);
	Joystick *gamePad =new Joystick(2);

	frc::Compressor *garry= new Compressor(0);
	frc::DoubleSolenoid *arm =new DoubleSolenoid(0,1);

	std::string gameData;

	frc::RobotDrive *robotDrive =new frc::RobotDrive (fLeft,bLeft,fRight,bRight);

	void RobotInit() {

		arm->Set(frc::DoubleSolenoid::kReverse);
		cam.SetBrightness(1200);
		cam.SetExposureManual(42);
		cam.SetWhiteBalanceManual(3800);
		chooser.AddDefault(autoNormal,autoNormal);
		chooser.AddObject(autoForward, autoForward);
		chooser.AddObject(autoNone, autoNone);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

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

	void AutonomousInit() override{
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		autoSelected = chooser.GetSelected();
		std::cout << "Auto selected: " << autoSelected << std::endl;

		encLeft->Reset();
		encRight->Reset();

	}

	void AutonomousPeriodic() {
		lDis=encLeft->GetRaw();
		rDis=encRight->GetRaw();
		//Forward
		if(autoSelected==autoNormal){
			if(lDis<=2011&&rDis<=2011){
				lDrive=.7;
				rDrive=.7;
			}
			else{
				lDrive=.7;
				rDrive=.7;
			}
		}
		//Forward

		robotDrive->TankDrive(lDrive,rDrive);
	}

	//AUTO END


	void TeleopInt() {
		// integrating time-to-forget
		garry->Enabled();
	}
	//TELE START
	void TeleopPeriodic() {
		lDrive=.7*leftStick->GetRawAxis(1);
		rDrive=.7*rightStick->GetRawAxis(1);
		robotDrive->TankDrive(lDrive,rDrive);
		climby = gamePad->GetRawAxis(1);

		if (fabs(climby) < .1) {
			climby = 0;
		}
		elevator->Set(climby);
		armout=gamePad->GetRawButton(3);
		armin=gamePad->GetRawButton(4);
		if(armout){
			arm->Set(frc::DoubleSolenoid::kReverse);
		}
		else if(armin){
			arm->Set(frc::DoubleSolenoid::kForward);
		}
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		SmartDashboard::PutNumber("Output", climby );
		SmartDashboard::PutNumber("Raw", gamePad->GetRawAxis(1));

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
 *		4 Elevator
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
