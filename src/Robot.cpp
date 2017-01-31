#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <WPILib.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>//all of these might not be neccisary


class Robot: public frc::IterativeRobot {//uncoment to enable vision

public:

	double Leftgo,Rightgo,Rdis,Ldis;
	bool   light,kickerswitcher;
	bool   SparkUno,kickatpos;
	bool   SparkDue;

	Joystick *rightDrive =new Joystick(0,2,9);
	Joystick *leftDrive  =new Joystick(1,2,9);
	Joystick *gamePad    =new Joystick(2,6,9);

	Talon *fLeft         =new Talon(0);
	Talon *fRight        =new Talon(1);
	Talon *bLeft         =new Talon(2);
	Talon *bRight        =new Talon(3);
	Spark *kicker             =new Spark(4);
	Spark *B             =new Spark(5);

	Encoder *encRight    =new Encoder(0,1);
	Encoder *encLeft     =new Encoder(2,3);
	Encoder *encKicker	=new Encoder(4,5);

	RobotDrive *robotDrive  =new RobotDrive(fLeft,fRight,bLeft,bRight);

	static void VisionThread(){// multithreading is required for the image proccessing so yah
		cs::UsbCamera cam =  CameraServer::GetInstance()->StartAutomaticCapture(0); //starts capturing basic images into camera
		//cs::UsbCamera cam2 = CameraServer::GetInstance()->StartAutomaticCapture(1);

		cam.SetResolution(320,240);
		// cam2.SetResolution(640,480);

		cs::CvSink sinker = CameraServer::GetInstance()->GetVideo(cam);//Grabs video to sink into the mat image cruncher

		cs::CvSource cheese = CameraServer::GetInstance()->PutVideo("Rectangle",640,480);//Serves up the images gathered your on camera

		cv::Mat cruncher(320,240,CV_8U);//this is the magic image cruncher when converting make sure there both the same type try this next CV_16UC1 its 16nit the other is 8
		// also there is CV_32FC1 i think its 32bit these also appere to not need the C1 so try CV_8U and the likes
		// cv::Mat cruncher2(320,240,CV_8U);
		std::vector<std::vector<cv::Point> > contours;

		while(true){//image processing happens in here


			if(sinker.GrabFrame(cruncher)==0){// if theres nothing there you got problems

				cheese.NotifyError(sinker.GetError());//HEY LISTEN! you got some problems tell me about them
				continue;//restarts the thread I think
			}

			cv::inRange(cruncher,cv::Scalar(0,100,0),cv::Scalar(100,255,100),cruncher);//finds them greens
			//cv::Canny(cruncher,cruncher,7,21,3);
		//	cv::blur(cruncher,cruncher,cv::Size(160,120),cv::Point(-1,-1));
			cv::findContours(cruncher,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);//draws external greens and stores them in contors
			cv::drawContours(cruncher,contours,-1,cv::Scalar(255,255,255),1,8);//draws dem contors

			cheese.PutFrame(cruncher);
			SmartDashboard::PutNumber("Point 25,25",cruncher.at<uchar>(25,25));
			// SmartDashboard::PutNumber("maxposn#1",max1posn);
		}
	}


	void RobotInit() {
		std::thread camthread(VisionThread);//makes a new thread
		camthread.detach();//snaps the thread off to do its own thing

		chooser.AddDefault(DOA, DOA);
		chooser.AddObject(NOTHING, NOTHING);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		encRight->Reset();
		encLeft->Reset();

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
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == NOTHING) {
			// Custom Auto goes here
		}
		else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		Rdis=encRight->GetRaw();
		Ldis=encLeft->GetRaw();
		if (autoSelected == NOTHING) {
			Rightgo=0;
			Leftgo=0;

		}

	//
		else {//DOA

			if(Rdis<=2160&&Ldis<=2160){//114.3" from wall to wall of airship ~6 rev
				Rightgo=.75;
				Leftgo=.75;
			}
			else{
				Rightgo=0;
				Leftgo=0;
			}
		}
	//
		robotDrive->TankDrive(Leftgo,Rightgo);
	}

	void TeleopInit() {
		Leftgo      =0;
		Rightgo     =0;
		light       =0;
		SparkUno    =0;
		SparkDue    =0;
		encRight->Reset();
		encLeft->Reset();

	}

	void TeleopPeriodic() {
//
		Leftgo =.75*leftDrive->GetRawAxis(1);
		Rightgo=.75*rightDrive->GetRawAxis(1);

		robotDrive->TankDrive(Leftgo,Rightgo);
//

		kickerswitcher   =gamePad->GetRawButton(1);
		SparkUno=gamePad->GetRawButton(2);
		SparkDue=gamePad->GetRawButton(3);


//
		if(kickerswitcher&&kickatpos){//if at pos and button
			kicker->Set(1);//move
			encKicker->GetRaw();//read enc
			if((encKicker->GetRaw())<=-15){//if at -15 deg call for reverse
				kickatpos=false;
			}
		}
		else if(!kickatpos){//reverse caller
			kicker->Set(-1);//backwards
			encKicker->GetRaw();//read enc
			if((encKicker->GetRaw())<=30){//if at 30 deg your home
				kickatpos=true;
			}
		}
		else{//else stop
			kicker->Set(0);
		}
//

		SmartDashboard::PutNumber("encRight",encRight->GetRaw());
		SmartDashboard::PutNumber("encLeft",encLeft->GetRaw());

		SmartDashboard::PutNumber("Leftgo",Leftgo);
		SmartDashboard::PutNumber("Rightgo",Rightgo);
	}

	void TestPeriodic() {
		lw->Run();
	}

private://why is this down here?
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string NOTHING = "NOTHING!";
	const std::string DOA = "FORWARD!";

	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
