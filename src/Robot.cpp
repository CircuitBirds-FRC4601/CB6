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
	float climby;
	int lDis=0,rDis=0;
	int encRes=1300;//Ticks per inch
	bool leg0,leg1,leg2,leg3,leg4;
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
		leg4=0;
		leg3=0;
		leg2=0;
		leg1=0;
		leg0=0;
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

		//Forward Box
		else if(autoSelected==autoForward){
			if(gameData.length>0&&gameData[0]=='L'){
				if(rDis<=140*encRes||lDis<=140*encRes){
					lDrive=.7;
					rDrive=.7;
				}
				else{
					lDrive=0;
					rDrive=0;
				}
			}
			else{
				lDrive=0;
				rDrive=0;
			}
		}
		//Forward Box

		//Magic
		else if(autoSelected==autoForward){
			if(gameData.length>0){
				if(gameData[0]=='L'){//Its in front!
					if(leg1&&(rDis<=10*encRes||lDis<=10*encRes)){
						lDrive=.7;
						rDrive=.7;
					}
					else{
						lDrive=0;
						rDrive=0;
					}
				}
				else{
					lDrive=0;
					rDrive=0;
				}
			}

			else{//Its to the right!
				if(!leg0){
					if(rDis<=10*encRes||lDis<=10*encRes){
						lDrive=.7;
						rDrive=.7;
					}
					else{
						lDrive=0;
						rDrive=0;
						leg0=1;
					}
				}
				else if(!leg1){//Spin!
					if(rDis<=10*encRes||lDis<=10*encRes){
						lDrive=.7;
						rDrive=-.7;
					}
					else{
						lDrive=0;
						rDrive=0;
						leg1=1;
					}
				}//Spin!
				else if(!leg2){//7' cross
					if(rDis<=84*encRes||lDis<=84*encRes){
						lDrive=.7;
						rDrive=.7;
					}
					else{
						lDrive=0;
						rDrive=0;
						leg1=1;
					}
				}
			}
		}
		//Magic

		SmartDashboard::PutNumber("Right Encoder", rDis);
		robotDrive->TankDrive(lDrive,rDrive);
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~AUTO END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TELE START~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	void TeleopInt() {
		// integrating time-to-forget
		garry->Enabled();
	}

	void TeleopPeriodic() {
		lDrive=.7*leftStick->GetRawAxis(1);
		rDrive=.7*rightStick->GetRawAxis(1);
		robotDrive->TankDrive(lDrive,rDrive);

		climby = gamePad->GetRawAxis(1);
		//foo
		if (fabs(climby) < .1) {
			climby = 0;
		}
		elevator->Set(climby);

		//Arm
		armout=gamePad->GetRawButton(3);
		armin=gamePad->GetRawButton(4);
		if(armout){
			arm->Set(frc::DoubleSolenoid::kReverse);
		}
		else if(armin){
			arm->Set(frc::DoubleSolenoid::kForward);
		}
		//Arm End

		SmartDashboard::PutNumber("Output", climby );
		SmartDashboard::PutNumber("Raw", gamePad->GetRawAxis(1));

	}
	void TestPeriodic() {
		lw.Run();
	}
private:
	frc::LiveWindow& lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoMagic= "Magic";//Use the FMS to make decisions.
	const std::string autoForward = "Just Forward";
	const std::string autoForwardBox = "Forward Box";//Goes forward if FMS says it can
	const std::string autoNone = "NONE";
	std::string autoSelected;

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
