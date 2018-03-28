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
private:
	//Auto Names
	frc::SendableChooser<std::string> chooser;
	const std::string autoMagic= "Magic";//Use the FMS to make decisions.
	const std::string autoForward = "FORWARD!";
	const std::string autoForwardBox = "Forward Box (Left)";//Goes forward and drops box if FMS says it can
	const std::string autoNone = "NONE";
	std::string autoSelected;
	//Auto Names

	static void VisionThread() {
		// Get the USB camera from CameraServer
		cs::UsbCamera camera =
				CameraServer::GetInstance()
		->StartAutomaticCapture();
		// Set the resolution
		camera.SetResolution(640, 480);
		camera.SetBrightness(1200);
		camera.SetExposureManual(42);

		// Get a CvSink. This will capture Mats from the Camera
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		// Setup a CvSource. This will send images back to the Dashboard
		cs::CvSource outputStream =
				CameraServer::GetInstance()->PutVideo(
						"Rectangle", 640, 480);

		// Mats are very memory expensive. Lets reuse this Mat.
		cv::Mat mat;

		while (true) {
			// Tell the CvSink to grab a frame from the camera and
			// put it
			// in the source mat.  If there is an error notify the
			// output.
			if (cvSink.GrabFrame(mat) == 0) {
				// Send the output the error.
				outputStream.NotifyError(cvSink.GetError());
				// skip the rest of the current iteration
				continue;
			}
			// Put a rectangle on the image
			// Give the output stream a new image to display
			outputStream.PutFrame(mat);
		}
	}


public:
	float lDrive=0,rDrive=0;
	float elevation,angle;
	int lDis=0,rDis=0;
	int encRes=56;//Encoder Resolution Ticks per inch
	bool armout,armin;
	bool climby,shotIn,shotOut,eStop;
	bool dumm,tiltdum=0,tilter,cable;

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
	frc::Encoder *encElevator =new Encoder(4,5);

	Relay *realy = new Relay(0,Relay::Direction::kForwardOnly);



	frc::Compressor *garry= new Compressor(0);
	frc::DoubleSolenoid *arm =new DoubleSolenoid(0,1);
	frc::DoubleSolenoid *tilt =new DoubleSolenoid(2,3);


	Joystick *leftStick =new Joystick(0);
	Joystick *rightStick =new Joystick(1);
	Joystick *gamePad =new Joystick(2);



	std::string gameData;//its a 3 letter String Depicting Sides;

	frc::RobotDrive *robotDrive =new frc::RobotDrive (fLeft,bLeft,fRight,bRight);

	void RobotInit() {
		//Auto Chooser
		chooser.AddDefault(autoForward,autoForward);
		chooser.AddDefault(autoForwardBox,autoForwardBox);
		chooser.AddObject(autoMagic, autoMagic);
		chooser.AddObject(autoNone, autoNone);
		frc::SmartDashboard::PutData("Auto Modes",&chooser);
		//Auto Chooser

		//Vision Detachment
		std::thread visionThread(VisionThread);
		visionThread.detach();
		//Vision Detachment
	}


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~AUTO START~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	void AutonomousInit() override{

		tilt->Set(frc::DoubleSolenoid::kForward);//PUT THE GUN DOWN! Puts arms down

		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();//Reads the magic field config


		autoSelected = chooser.GetSelected();//Grab the chosen Auto
		std::cout << "Auto selected: " << autoSelected << std::endl;



		encLeft->Reset();
		encRight->Reset();
		encElevator->Reset();

		rDis=0;
		lDis=0;
		dumm=false;
		leg0=false;
		leg1=false;
		leg2=false;
	}

	void AutonomousPeriodic() {
		lDis=encLeft->GetRaw();//Grabs Encoder Values
		rDis=encRight->GetRaw();

		//Forward Simple Easy
		if(autoSelected==autoForward){//10ft line + 1ft forward
			if(abs(rDis)<=132*encRes&&abs(lDis)<=132*encRes){
				lDrive=.7;
				rDrive=.7;
			}
			else{
				lDrive=0;
				rDrive=0;
			}
		}
		//Forward Simple Easy

		//****************************************Forward Box*****************************************************
		else if(autoSelected==autoForwardBox){//Goes Forward and sees if the FMS says it is on our side;
			if(!leg0&&abs(rDis)<=168*encRes&&abs(lDis)<=168*encRes){
				lDrive=.7;
				rDrive=.7;
			}


			else if(!leg0){//Reset for SPIN!
				encLeft->Reset();
				encRight->Reset();
				leg0=true;//First leg finished on to 2;
			}



			else if(!leg1){//SPIN Check
				if(gameData.length()>0){//make sure there is data
					if(gameData[0]=='L'){//Is the Switch Left?
						if(abs(rDis)<=25*encRes&&abs(lDis)<=25*encRes){//SPIN! 90deg spin ~24.7in
							lDrive=.65;
							rDrive=-.65;
						}
						else{//PERISCOPE UP!
							lDrive=0;
							rDrive=0;
							if((encElevator->GetRaw())<=36*encRes){//Raise the box up it only needs to go up 18.75in NEEDS CALIBRATED!!!!
								realy->Set(Relay::kOn);
								elevator->Set(.75);
							}
							else{
								realy->Set(Relay::kOff);
								elevator->Set(0);
								encLeft->Reset();
								encRight->Reset();
								leg1=true;//We Done Spinning!
							}
						}
					}

					else{//It is not on the Left STOP!
						leg2=1;
						leg1=1;
					}
				}

				else{//NO DATA AHHHHHHHHHHHHHHHHH ERRRRRRROOOOOORRR!!!!!
					leg2=1;
					leg1=1;
				}

			}


			else if(!leg2){//Final distance
				if(abs(rDis)<=50*encRes&&abs(lDis)<=50*encRes){//55.56inch some wiggly room so we dont slam into it
					lDrive=.7;
					rDrive=.7;
				}
				else{
					shooter->Set(1);
					sleep(2);//give the box time to get out of there
					arm->Set(frc::DoubleSolenoid::kReverse);//DROP DA BOMB!!!!
					shooter->Set(0);
					leg2=true;
				}
			}

			else{//*Sigh* and now we are done
				lDrive=0;
				rDrive=0;
			}
		}
		//****************************************Forward Box*****************************************************


		//Print Encoder Values
		SmartDashboard::PutNumber("Right Encoder", rDis);
		SmartDashboard::PutNumber("Right Encoder", lDis);
		//Print Encoder Values

		robotDrive->TankDrive(lDrive,rDrive);//Drives based on previous Drive values
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~AUTO END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~TELE START~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	void TeleopInt() {
		garry->Enabled();
		dumm=0;
	}

	void TeleopPeriodic() {
		//DRIVE

		lDrive=-.7*leftStick->GetRawAxis(1);
		rDrive=-.7*rightStick->GetRawAxis(1);
		robotDrive->TankDrive(lDrive,rDrive);

		//DRIVE END

		//Elevator and Climber
		elevation = gamePad->GetRawAxis(1);
		if (fabs(elevation) < .1) {
			elevation = 0;
		}
		if(elevation<0.0){
			realy->Set(Relay::kOn);
		}
		else{
			realy->Set(Relay::kOff);
		}



		if(!dumm) {
			elevator->Set(.75*elevation);
		}
		else{
			elevator->Set(0);
		}

		climby=gamePad->GetRawButton(8);
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

		if(gamePad->GetRawButton(7)&&!tiltdum){
			tilter=!tilter;
		}
		if(tilter){
			tilt->Set(frc::DoubleSolenoid::kForward);
		}
		else{
			tilt->Set(frc::DoubleSolenoid::kReverse);
		}
		tiltdum=gamePad->GetRawButton(7);

		//Tilter END

		//BOX GRABBER

		SmartDashboard::PutNumber("Left",encLeft->GetRaw());
		SmartDashboard::PutNumber("Right", encRight->GetRaw());

	}
	void TestPeriodic() {

	}

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
