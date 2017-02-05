#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <WPILib.h>
#include <CameraServer.h>
#include <unistd.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>//all of these might not be neccisary


class Robot: public frc::IterativeRobot {

public:

	double Leftgo,Rightgo,Rdis,Ldis;
	double climbspeed,light,push;
	bool   kickerswitcher;
	bool   kickerdummy,kickerrunning;
	bool   greenholder;


	Joystick *rightDrive =new Joystick(0,2,9);
	Joystick *leftDrive  =new Joystick(1,2,9);
	Joystick *gamePad    =new Joystick(2,6,9);

	Talon *fLeft         =new Talon(0);
	Talon *fRight        =new Talon(1);
	Talon *bLeft         =new Talon(2);
	Talon *bRight        =new Talon(3);
	Spark *kicker        =new Spark(4);
	Spark *climber       =new Spark(5);
	Spark *frankenspark  =new Spark(6);

	Encoder *encRight    =new Encoder(0,1);
	Encoder *encLeft     =new Encoder(2,3);
	Encoder *encKicker	 =new Encoder(4,5);

	RobotDrive *robotDrive  =new RobotDrive(fLeft,fRight,bLeft,bRight);

	//VISION DECL START

		//Auto Camera
	cs::UsbCamera cam2		= CameraServer::GetInstance()->StartAutomaticCapture(1);//sets up camera 2 for capturing
			cs::CvSource cheese		= CameraServer::GetInstance()->PutVideo("Rectangle",640,480);//creates a video stream called rectangle
			cs::CvSink autosinker	= CameraServer::GetInstance()->GetVideo(cam2);//attach the sinker to the camera
		//Auto Camera

		//Matrixes
		cv::Mat pregreen 	= cv::Mat(640,480,CV_8U);
		cv::Mat green 		= cv::Mat(640,480,CV_8U);
		//Matrixes

		//Points and lines
	//	std::vector<std::vector<cv::Point> > contours;
		//cv::Point point1,point2;
		//cv::Rect  rect1;
		//Points and lines

	//VISION DECL END

	/*static void VisionThread(){// multithreading is required for the image proccessing so yah
		cs::UsbCamera cam =  CameraServer::GetInstance()->StartAutomaticCapture(0); //starts capturing basic images into camera
		//cs::UsbCamera cam2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
		cam.SetBrightness(.5);
		cam.SetExposureManual(-11);
		cam.SetResolution(640,480);
		// cam2.SetResolution(640,480);

		cs::CvSink sinker = CameraServer::GetInstance()->GetVideo(cam);//Grabs video to sink into the mat image cruncher

	//	cs::CvSource cheese = CameraServer::GetInstance()->PutVideo("Rectangle",640,480);//Serves up the images gathered your on camera

		cv::Mat cruncher(640,480,CV_8U);
		std::vector<std::vector<cv::Point> > contours;

		while(true){//image processing happens in here


			if(sinker.GrabFrame(cruncher)==0){// if theres nothing there you got problems

				//cheese.NotifyError(sinker.GetError());//HEY LISTEN! you got some problems tell me about them
				continue;//restarts the thread I think
			}

		//	cv::inRange(cruncher,cv::Scalar(25,20,15),cv::Scalar(40,50,30),cruncher);//finds them greens
			//cv::erode(cruncher,cruncher,)
			//cv::Canny(cruncher,cruncher,7,21,3);
		//	cv::blur(cruncher,cruncher,cv::Size(160,120),cv::Point(-1,-1));
		//	cv::findContours(cruncher,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);//draws external greens and stores them in contors
			//cv::drawContours(cruncher,contours,-1,cv::Scalar(255,255,255),1,8);//draws dem contors

			//cheese.PutFrame(cruncher);
			SmartDashboard::PutNumber("Point 25,25",cruncher.at<uchar>(25,25));

			// SmartDashboard::PutNumber("maxposn#1",max1posn);
		}
	}*/


	void RobotInit() {
		//std::thread camthread(VisionThread);//makes a new thread
		//camthread.detach();//snaps the thread off to do its own thing

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
			greenholder=0;
			cam2.SetBrightness(100);
			cam2.SetExposureManual(0);
			cam2.SetResolution(640,480);


		}
		else {
			// Default Auto goes here
		}
	}

//AUTO START

	void AutonomousPeriodic() {
		Rdis=encRight->GetRaw();
		Ldis=encLeft->GetRaw();

		//Nothing
		if (autoSelected == NOTHING) {//sit there yah lazy bum
			Rightgo=0;
			Leftgo=0;

			//
			if(!greenholder){
				autosinker.GrabFrame(pregreen);//grabs a pregreen image
				frankenspark->Set(-1);//turn on the lights
				sleep(5);//1 sec delay for light to turn on
				autosinker.GrabFrame(green);//grabs a green image
				frankenspark->Set(0);//turns off light
				greenholder=1;
			}
			//

			else if(greenholder&&!push){
				//cv::addWeighted(pregreen,4,green,-4,0,green);//meshes pregreen and green then outputs to green
				cv::inRange(green,cv::Scalar(255,0,255),cv::Scalar(255,255,255),green);//does some BGR thresholds on Mat green
			//	cv::medianBlur(green,green,23);//blurs to remove noise with "radius" of 23 pixels (Its kernel size AKA mat size)
				//cv::findContours(green,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);//look into this line, check arguments. Finds contours in mat green then puts them in contours

				//rect1 =	cv::boundingRect(contours[0]);//puts the bounding rectangle of original contour in rect1
				//we are probably going to need to filter these contours

				//point1.x = rect1.x;//grabs x from rect1
				//point1.y = rect1.y;//grabs y from rect1
				//point2.x = rect1.x+rect1.width;//grabs the opposite corner
			//	point2.y = rect1.y+rect1.height;

			//	cv::rectangle(green,point1,point2,cv::Scalar(255,0,0),5);//draws rectangle with point1 and point2

				cheese.PutFrame(green);
			}
			else{

			}

		}
		//Nothing

		//DOA
		else {//Dead On Arrival AKA Dead Reckoning

			if(Rdis<=2160&&Ldis<=2160){//114.3" from wall to wall of airship ~6 rev
				Rightgo=.75;
				Leftgo=.75;
			}
			else{
				Rightgo=0;
				Leftgo=0;
			}
		}
		//DOA

		robotDrive->TankDrive(Leftgo,Rightgo);
	}

//AUTO END

	void TeleopInit() {
		Leftgo      =0;
		Rightgo     =0;
		light       =0;
		encRight->Reset();
		encLeft->Reset();

	}

//TELE START

	void TeleopPeriodic() {

		//Drive
		Leftgo =.75*leftDrive->GetRawAxis(1);
		Rightgo=.75*rightDrive->GetRawAxis(1);

		robotDrive->TankDrive(Leftgo,Rightgo);
		//Drive

		//Kicker
		if(!kickerrunning){
			kickerswitcher   =gamePad->GetRawButton(1);
		}

		if(kickerswitcher&&!kickerdummy){//Forward
			kickerrunning=1;
			kicker->Set(-.5);//Move Forwards
			encKicker->GetRaw();//Read Enc
			if((encKicker->GetRaw())>=350){
				kickerrunning=0;//No Longer Running
				kickerdummy=1;
				kicker->StopMotor();
			}
		}
		else if(kickerswitcher&&kickerdummy){//Reverse
			kickerrunning=1;
			kicker->Set(.5);//Move Backwards
			encKicker->GetRaw();//Read Enc
			if((encKicker->GetRaw())<=100){//Stop Early to Comp for Drift
				kickerrunning=0;//No Longer Running
				kickerdummy=0;
				kicker->StopMotor();
			}
		}
		else{//Stop if no button
			kicker->Set(0);
		}

		SmartDashboard::PutNumber("enckicker",encKicker->GetRaw());
		//Kicker

		//Climber
		climbspeed=gamePad->GetRawAxis(0);
		if(fabs(climbspeed)>=.5){
			climber->Set(climbspeed);
		}
		else{
			climber->Set(0);
		}
		//Climber

		//Light
		light= gamePad->GetRawAxis(4);
		if(fabs(light)>=0.25){
			frankenspark->Set(-fabs(light));//have to do - for Doc's franken spark
		}
		else{
			frankenspark->Set(0);
		}
		SmartDashboard::PutNumber("light",fabs(light));
		//Light

		//SmartDashboard
		SmartDashboard::PutNumber("climbspeed",climbspeed);
		SmartDashboard::PutNumber("encRight",encRight->GetRaw());
		SmartDashboard::PutNumber("encLeft",encLeft->GetRaw());

		SmartDashboard::PutNumber("Leftgo",Leftgo);
		SmartDashboard::PutNumber("Rightgo",Rightgo);
		//SmartDashboard
	}

//TELE END

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
/* Hardware map of the robot "TBA"  (CB5)
 *
 * 		PWM
 *		0 Front Left
 *		1 	"	Right
 *		2 Back	Left
 *		3	"	Right
 *		4 Kicker
 *		5 Climber
 *		6 Franken Spark
 *		7
 *		8
 *		9
 *
 * 1ft=~720 check this
 *  RRio Pins
 *  	DIO
 *  	0	A Right Wheel Encoder
 *  	1	B "
 *  	2	A Left Wheel Encoder
 *  	3	B "
 *  	4	A Kicker Encoder
 *  	5	B "
 *  	6
 *  	7
 *  	8
 *  	9
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
