#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <WPILib.h>
#include <LiveWindow/LiveWindow.h>
#include <LiveWindow/LiveWindow.h>
#include <WPILib.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CameraServer.h>
#include <unistd.h>
#include <IterativeRobot.h>
#include <opencv2/core/core.hpp>
#include <Encoder.h>

class Robot: public frc::IterativeRobot {

public:
	float lDrive=0,rDrive=0;
	float elevation,angle;
	int lDis=0,rDis=0;
	int encRes=50;//Ticks per inch
	bool armout,armin;
	bool climby,shotIn,shotOut,eStop;
	bool dumm,tiltdum,tilter;

	bool leg0,leg1,leg2,leg3,leg4;


	Spark *shooter =new Spark(0);
	Spark *angler =new Spark(1);
	Spark *fRight =new Spark(2);
	Spark *bRight =new Spark(3);
	Spark *climber =new Spark(4);
	Spark *elevator =new Spark(5);
	Spark *fLeft =new Spark(6);
	Spark *bLeft =new Spark(7);

	frc::Encoder *encLeft  =new Encoder(0,1);
	frc::Encoder *encRight =new Encoder(2,3);
	frc::Encoder *encClimb =new Encoder(4,5);
	DigitalInput *limit    =new DigitalInput(6);

	cs::UsbCamera cam= CameraServer::GetInstance()->StartAutomaticCapture();

	Joystick *leftStick =new Joystick(0);
	Joystick *rightStick =new Joystick(1);
	Joystick *gamePad =new Joystick(2);

	frc::Compressor *garry= new Compressor(0);
	frc::DoubleSolenoid *arm =new DoubleSolenoid(0,1);
	frc::DoubleSolenoid *tilt =new DoubleSolenoid(2,3);

	std::string gameData;

	frc::RobotDrive *robotDrive =new frc::RobotDrive (fLeft,bLeft,fRight,bRight);

	void RobotInit() {
		cam.SetBrightness(1200);
		cam.SetExposureManual(42);
		cam.SetWhiteBalanceManual(3800);
		chooser.AddDefault(autoForward,autoForward);
		chooser.AddDefault(autoForwardBox,autoForwardBox);
		chooser.AddObject(autoMagic, autoMagic);
		chooser.AddObject(autoNone, autoNone);
		frc::SmartDashboard::PutData("Auto Modes",&chooser);
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
		if(autoSelected==autoForward){

			if(lDis<=140*encRes || rDis<=140*encRes){
				lDrive = .7;
				rDrive = .7;

			}
			else{
				lDrive = 0;
				rDrive = 0;
			}
		}
		//Forward
		else if(autoSelected==autoForward){

			if(lDis<=140*encRes || rDis<=140*encRes){
				lDrive = .7;
				rDrive = .7;

			}
			else{
				lDrive = 0;
				rDrive = 0;
			}
		}

		SmartDashboard::PutNumber("Right Encoder", rDis);
		SmartDashboard::PutNumber("Right Encoder", lDis);
		robotDrive->TankDrive(lDrive,rDrive);
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~AUTO END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TELE START~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	void TeleopInt() {
		garry->Enabled();
	}

	void TeleopPeriodic() {
		//DRIVE

		lDrive=-.7*leftStick->GetRawAxis(1);
		rDrive=-.7*rightStick->GetRawAxis(1);
		robotDrive->TankDrive(lDrive,rDrive);

		//DRIVE END
		/*if(gamePad->GetRawButton(8)){
			encLeft->Reset();
			encRight->Reset();
			dumm=0;
			lDis=0;
			rDis=0;
			while(!dumm){
				lDis=encLeft->GetRaw();
				rDis=encRight->GetRaw();
				if(rDis<2000&&lDis<2000){
					lDrive=.7;
					rDrive=.7;
				}
				else{
					lDrive=0;
					rDrive=0;
					dumm=1;
				}
				robotDrive->TankDrive(lDrive,rDrive);
				SmartDashboard::PutNumber("Left",encLeft->GetRaw());
				SmartDashboard::PutNumber("Right", encRight->GetRaw());
			}
		}*/
		//Elevator and Climber
		elevation = gamePad->GetRawAxis(1);
		if (!dumm&&fabs(elevation) < .1) {
			elevation = 0;
		}
		else if(!dumm) {
			elevator->Set(.75*elevation);
			if(elevator->Get()<0){
				//catch.off;
			}
		}
		else{
			elevator->Set(0);
		}

		climby=gamePad->GetRawButton(7);
		if(climby){
			climber->Set(1);
			dumm=1;
		}
		else{
			climber->Set(0);
			dumm=0;

		}
		//Elevator and Climber END

		//BOX GRABBER

		//Arm
		armout=gamePad->GetRawButton(1);
		armin=gamePad->GetRawButton(2);
		if(armout){
			arm->Set(frc::DoubleSolenoid::kReverse);
		}
		else if(armin){
			arm->Set(frc::DoubleSolenoid::kForward);
		}
		//Arm End

		//Shooter
		shotIn=gamePad->GetRawAxis(2);
		shotOut=gamePad->GetRawAxis(3);
		if(shotIn){
			shooter->Set(-1);
		}
		else if(shotOut){
			shooter->Set(1);
		}
		else{
			shooter->Set(0);
		}
		//Shooter END

		//Tilter

		if(gamePad->GetRawButton(6)&&!tiltdum){
			tilter=!tilter;
		}
		if(gamePad->GetRawButton(6)){
			tiltdum=1;
		}
		else{
			tiltdum=0;
		}
		if(tilter){
		tilt->Set(frc::DoubleSolenoid::kForward);
		}
		else{
			tilt->Set(frc::DoubleSolenoid::kReverse);
		}
		//Tilter END

		//BOX GRABBER

		SmartDashboard::PutNumber("Left",encLeft->GetRaw());
		SmartDashboard::PutNumber("Right", encRight->GetRaw());

	}
	void TestPeriodic() {

	}
private:

	frc::SendableChooser<std::string> chooser;
	const std::string autoMagic= "Magic";//Use the FMS to make decisions.
	const std::string autoForward = "Just Forward";
	const std::string autoForwardBox = "Forward Box";//Goes forward if FMS says it can
	const std::string autoNone = "NONE";
	std::string autoSelected;

};

START_ROBOT_CLASS(Robot)

/* Hardware map of the robot "TBA"  (CB5)
 *	1in= ~56 Wheel Encoders
 *		Game Pad
 *		Axis
 *		0 LX) Elevator
 *		1 LY)
 *		2 LTrig) Shooter In
 *		3 RTrig) Shooter Out
 *		4 RX)
 *		5 RY)
 *
 *		Button
 *		1 A) Arm Out
 *		2 B) Arm In
 *		3 X)
 *		4 Y)
 *		5 LB)
 *		6 RB)
 *		7 BCK) Tilt
 *		8 STR) Climb
 *		9 LSTK)
 *		10 RSTK)
 *
 *		RRio Pins
 * 		PWM
 *		0 Shooter
 *		1 Angler
 *		2 Front	Right
 *		3 Back	"
 *		4 Climber
 *		5 Elevator
 *		6 Front Left
 *		7 Back	"
 *		8
 *		9
 *
 *
 *  	DIO
 *  	0 Left Encoder  A
 *  	1 	   "		B
 *  	2 Right Encoder A
 *  	3	   "		B
 *  	4 	Climber     A
 *  	5 	   "	 	B
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
