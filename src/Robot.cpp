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

	int ii, jj, stripe_width, num_rows, num_columns, stripe_start_row, diff_int[641], integral[641];
	int max_integral,maxposn1,maxposn2, min_integral,minposn1,minposn2, tempi, cutoff_intensity, flagi, done_int;
	int arm_max=355, arm_set_up=73, arm_set_down=349, byte;
	double Leftgo,Rightgo,Rdis,Ldis;
	bool forwardReach,backUp;
	double climbspeed,shotspeed,light,push, heading, headinglast;
	bool   kickerdown,kickerup,kickerEreset;
	bool   kickerdummy,kickerrunning;
	bool   greenholder, superdum, stop_arm1, state;


	Joystick *rightDrive =new Joystick(0,2,9);
	Joystick *leftDrive  =new Joystick(1,2,9);
	Joystick *gamePad    =new Joystick(2,6,9);

	Spark *fLeft         =new Spark(0);
	Spark *fRight        =new Spark(1);
	Spark *bLeft         =new Spark(2);
	Spark *bRight        =new Spark(3);
	Talon *kicker        =new Talon(4);
	Talon *climber       =new Talon(5);
	Spark *frankenspark  =new Spark(6);
	Talon *shooter		 =new Talon(7);

	Encoder *encRight    =new Encoder(0,1);
	Encoder *encLeft     =new Encoder(2,3);
	Encoder *encKicker	 =new Encoder(4,5);

	DigitalInput *limitArm = new DigitalInput(6);//reads the arm limit switch
	DigitalOutput *SDC = new DigitalOutput(7);//
	DigitalInput *SDA = new DigitalInput(8);//

	frc::ADXRS450_Gyro *gyro =new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);


	RobotDrive *robotDrive  =new RobotDrive(fLeft,fRight,bLeft,bRight);

	//VISION DECL START

	//Auto Camera
	cs::UsbCamera cam		= CameraServer::GetInstance()->StartAutomaticCapture(0);//sets up camera for capturing

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

		cam2.SetBrightness(150);
		cam2.SetExposureManual(-6);
		cam2.SetWhiteBalanceManual(2800);
		cam2.SetResolution(640,480);

		cam.SetBrightness(150);
		cam.SetExposureManual(-6);
		cam.SetWhiteBalanceManual(2800);
		cam.SetResolution(640,480);

		chooser.AddDefault(NOTHING, NOTHING);
		chooser.AddObject(DOA, DOA);
		chooser.AddObject(Light, Light);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		encRight->Reset();
		encLeft->Reset();
		gyro->Calibrate();

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

		stop_arm1=limitArm->Get();

			while(!stop_arm1){
			stop_arm1=limitArm->Get();
			kicker->Set(.25);
		}
		kicker->Set(0);
		encKicker->Reset();



		if (autoSelected == NOTHING) {
			// Custom Auto goes here
			greenholder=0;
			push=0;


		}
		else {
			forwardReach=0;
			kickerdummy=0;
			backUp=0;
			// Default Auto goes here
		}
	}

	//AUTO START

	void AutonomousPeriodic() {
		Rdis=encRight->GetRaw();
		Ldis=-(encLeft->GetRaw());
		//Nothing
		if (autoSelected == NOTHING) {//sit there yah lazy bum
			Rightgo=0;
			Leftgo=0;

			//
		}
		//Nothing

		//Light
		else if(autoSelected == Light){
			if(!greenholder){
				autosinker.GrabFrame(pregreen);//grabs a pregreen image
				frankenspark->Set(-1);//turn on the lights
				sleep(2.5);//1 sec delay for light to turn on
				autosinker.GrabFrame(green);//grabs a green image
				frankenspark->Set(0);//turns off light
				greenholder=1;
			}
			//
			else if(greenholder&&!push){
				cv::addWeighted(green,.9,pregreen,-1,0,green);//meshes pregreen and green then outputs to green Needs to be values of 1
				//cv::subtract(green,pregreen,green);
				//cv::inRange(green,cv::Scalar(255,0,255),cv::Scalar(255,255,255),green);//does some BGR thresholds on Mat green
				//cv::medianBlur(green,green,7);//blurs to remove noise with "radius" of 23 pixels (Its kernel size AKA mat size)
				//cv::findContours(green,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);//look into this line, check arguments. Finds contours in mat green then puts them in contours


				//rect1 =	cv::boundingRect(contours[0]);//puts the bounding rectangle of original contour in rect1
				//we are probably going to need to filter these contours

				//point1.x = rect1.x;//grabs x from rect1
				//point1.y = rect1.y;//grabs y from rect1
				//point2.x = rect1.x+rect1.width;//grabs the opposite corner
				//	point2.y = rect1.y+rect1.height;

				//	cv::rectangle(green,point1,point2,cv::Scalar(255,0,0),5);//draws rectangle with point1 and point2
				push=1;
			}
			else{
				cheese.PutFrame(green);
			}

		}
		//Light

		//DOA
		else {//Dead On Arrival AKA Dead Reckoning

			if(!forwardReach&&Rdis<=6117.67&&Ldis<=6117.67){//114.3" from wall to wall of airship ~6.92 feet
				Rightgo=-.75;
				Leftgo=-.75;
			}
			else if(!forwardReach){
				forwardReach=1;
				Rightgo=0;
				Leftgo=0;
			}

			if(forwardReach){

				if(!backUp&&!kickerdummy){//Forward
					kicker->Set(.5*((encKicker->GetRaw())-arm_set_down)/arm_max-0.5);//Move Forwards PID
					if((encKicker->GetRaw())>=301){
						kickerdummy=1;
						kicker->StopMotor();
						encRight->Reset();
						encLeft->Reset();
						sleep(1.5);
					}
				}
				else if(!backUp){
					Rightgo=.25;
					Leftgo=.25;
					if(fabs(encRight->GetRaw())>=1325.95){
						backUp=1;
						Rightgo=0;
						Leftgo=0;
					}
				}
				else if(kickerdummy){//Reverse
					kicker->Set(0.75*((encKicker->GetRaw())-arm_set_up)/arm_max+0.05);//Move Backwards PID and slows dows
					if((encKicker->GetRaw())<=61){//Stop Early to Comp for Drift
						kickerdummy=0;
						kicker->StopMotor();
					}
				}
				else{//stop
					Rightgo=0;
					Leftgo=0;

				}

			}

	}
		//DOA

		robotDrive->TankDrive(Leftgo,Rightgo);
		SmartDashboard::PutNumber("encRight",Rdis);
		SmartDashboard::PutNumber("encLeft", Ldis);
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
		//Video Practice
		superdum=gamePad->GetRawButton(7);
		greenholder=0;
		push=0;
		while(superdum){
			if(!greenholder){
				autosinker.GrabFrame(pregreen);//grabs a pregreen image
				frankenspark->Set(-1);//turn on the lights
				sleep(2.5);//1 sec delay for light to turn on
				autosinker.GrabFrame(green);//grabs a green image
				frankenspark->Set(0);//turns off light
				greenholder=1;
			}
			//
			else if(greenholder&&!push){
				cv::addWeighted(green,.9,pregreen,-1,0,green);//meshes pregreen and green then outputs to green Needs to be values of 1
				cv::threshold(green,green,0,0,cv::THRESH_TOZERO);
				//cv::subtract(green,pregreen,green);
				//cv::inRange(green,cv::Scalar(255,0,255),cv::Scalar(255,255,255),green);//does some BGR thresholds on Mat green
				//cv::medianBlur(green,green,7);//blurs to remove noise with "radius" of 23 pixels (Its kernel size AKA mat size)
				//cv::findContours(green,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);//look into this line, check arguments. Finds contours in mat green then puts them in contours

				//rect1 =	cv::boundingRect(contours[0]);//puts the bounding rectangle of original contour in rect1
				//we are probably going to need to filter these contours

				//point1.x = rect1.x;//grabs x from rect1
				//point1.y = rect1.y;//grabs y from rect1
				//point2.x = rect1.x+rect1.width;//grabs the opposite corner
				//	point2.y = rect1.y+rect1.height;

				//	cv::rectangle(green,point1,point2,cv::Scalar(255,0,0),5);//draws rectangle with point1 and point2
				push=1;
				// Dr. C.'s codelines for integrated luminosity lines
				num_columns = 640;
				num_rows = 480;
				stripe_start_row = 120;
				stripe_width = 40;
				max_integral = 0;
				min_integral = 0;
				cutoff_intensity = 13;
				for(ii=2; ii<num_columns; ii++){
					integral[ii] = 0;
					for(jj=stripe_start_row; jj<stripe_start_row+stripe_width; jj++){
						integral[ii] = integral[ii]+green.at<uchar>(ii,jj)+abs(green.at<uchar>(ii,jj));
					}
					if (integral[ii]<cutoff_intensity){
						integral[ii] = 0;      //assume that we reach zero somewhere inbetween the stripes
					}
					diff_int[ii] = integral[ii]-integral[ii-1]; // find the boundaries
					tempi = diff_int[ii];      //temporary integer; speeds up lookup in next lines.
					if(max_integral<tempi){    //if new value exceeds old, max set to new max
						maxposn1 = ii;         //gets value of global max
						max_integral=tempi;
					}
					if(min_integral>tempi){    //if new value exceeds old min, set to new global min
						minposn1 = ii;
						min_integral=tempi;
					}
				}
				max_integral=0; 				    //reuse these
				min_integral=0;
				done_int = 0; 						// flag to say when done with maxs/mins in order.
				if (maxposn1>minposn1){            // here put the extrema in order.
					maxposn2 = maxposn1;
					for(ii=2; ii<minposn1; ii++){
						tempi = diff_int[ii];
						if(max_integral<tempi){    //if new value exceeds old, max set to new max
							maxposn1 = ii;         //gets value of max
							max_integral=tempi;
						}
					}
					for(ii=maxposn2; ii<num_rows; ii++){
						tempi = diff_int[ii];
						if(min_integral>tempi){    //if new value exceeds old min, set to new min
							minposn2 = ii;
							min_integral=tempi;
						}
					}
					done_int = 1;                  // signal all done.
				}
				if ((maxposn1<minposn1)&&(done_int==0)) { // in this configuration all could be fine IF there is no zero between the posn...
					flagi = 0;
					for(ii=maxposn1; ii<minposn1; ii++){
						if(integral[ii]==0){
							flagi=1;               // SO they are not in the canonical order!
						}
					}
					if(flagi==0){					//cannonical order, find others and quit
						for(ii=minposn1; ii<num_rows; ii++){
							tempi = diff_int[ii];      //temporary integer; speeds up lookup in next lines.
							if(max_integral<tempi){    //if new value exceeds old, max set to new max
								maxposn2 = ii;         //gets value of global max
								max_integral=tempi;
							}
							if(min_integral>tempi){    //if new value exceeds old min, set to new global min
								minposn2 = ii;
								min_integral=tempi;
							}
						}
						if(maxposn2>minposn2){
							done_int=0;                 // still something is screwed up. Slew the 'bot and try again
						}
						else{
							done_int=1;
						}
					}
					if(flagi==1){
						minposn2=minposn1;
						for(ii=maxposn1; ii<minposn2; ii++){
							tempi = diff_int[ii];      //temporary integer; speeds up lookup in next lines.
							if(max_integral<tempi){    //if new value exceeds old, max set to new max
								maxposn2 = ii;         //gets value of global max
								max_integral=tempi;
							}
							if(min_integral>tempi){    //if new value exceeds old min, set to new global min
								minposn1 = ii;
								min_integral=tempi;
							}
						}
						if(maxposn2<minposn1){
							done_int=0;                 // still something is screwed up. Slew the 'bot and try again
						}
						else{
							done_int=1;
						}
					}
				}
				// at this point should be all done. Can check done_int=1 and if good you should have the ordered set
				//    (maxposn1, minposn1, maxposn2, minposn2) of the pixel numbers of the stripe edges!!
				// end of Dr. C.'s lines.


			}
			else{

				SmartDashboard::PutNumber("Computer Vision Success", done_int);
				SmartDashboard::PutNumber("camera first stripe outer edge", maxposn1);
				SmartDashboard::PutNumber("Camera first stripe inner edge", minposn1);
				SmartDashboard::PutNumber("camera second stripe inner edge", maxposn2);
				SmartDashboard::PutNumber("Camera second stripe outer edge", minposn2);

				//rectangle(green, cv::Point(0, stripe_start_row), cv::Point(640, stripe_start_row+stripe_width),cv::Scalar(255, 255, 255), 5);
				cheese.PutFrame(green);
				superdum=0;

			}
		}
		//Video Practice

		/* // Dr. C. lines MAGNETOMETER START
		byte=1;
		heading=0;
		state = 0;
		for(ii=0; ii<8; ii++){
			SDC->Set(state);
			SDC->Set(!state);
			heading = heading+byte*(SDA->Get());
			byte=byte*2;
		}
		SmartDashboard::PutNumber("Magnetometer", heading);
		// Dr. C.  MAGNETOMETER STOP */

		//Drive
		Leftgo =.75*leftDrive->GetRawAxis(1);
		Rightgo=.75*rightDrive->GetRawAxis(1);
		Rdis=encRight->GetRaw();
		Ldis=-(encLeft->GetRaw());

		robotDrive->TankDrive(Leftgo,Rightgo);
		//Drive

		//Kicker
		stop_arm1=limitArm->Get();
		if(!kickerEreset){
			kickerEreset=gamePad->GetRawButton(8);
		}

		if(kickerEreset){
			if(!stop_arm1){
				kicker->Set(.25);
			}
			else{
				kicker->Set(0);
				encKicker->Reset();
				kickerrunning=0;
				kickerdummy=0;
				kickerEreset=0;
			}
		}
		if(!kickerEreset){
			if(!kickerrunning){
				kickerdown   =gamePad->GetRawButton(1);
				kickerup   =gamePad->GetRawButton(2);
			}

			if(kickerdown&&!kickerdummy){//Forward
				kickerrunning=1;
				kicker->Set(.5*((encKicker->GetRaw())-arm_set_down)/arm_max-0.5);//Move Forwards PID
				if((encKicker->GetRaw())>=arm_set_down){
					kickerrunning=0;//No Longer Running
					kickerdummy=1;
					kicker->StopMotor();
				}
			}
			else if(kickerup&&kickerdummy){//Reverse
				kickerrunning=1;
				kicker->Set(.95*((encKicker->GetRaw())-arm_set_up)/arm_max+0.1);//Move Backwards PID and slows dows
				if((encKicker->GetRaw())<=arm_set_up){//Stop Early to Comp for Drift
					kickerrunning=0;//No Longer Running
					kickerdummy=0;
					kicker->Set(0);
				}
			}
			else{//Stop if no button
				kicker->Set(0);
			}
		}
		SmartDashboard::PutNumber("enckicker",encKicker->GetRaw());
		//Kicker

		//Shooter
		//shotspeed=gamePad->GetRawAxis(3);
		//shooter->Set(shotspeed);
	/*	if(encShooter>=357){
		gamePad->SetRumble(Joystick::RumbleType::kRightRumble,1);
		gamePad->SetRumble(Joystick::RumbleType::kLeftRumble,1);
		}
		else{
			gamePad->SetRumble(Joystick::RumbleType::kRightRumble,0);
			gamePad->SetRumble(Joystick::RumbleType::kLeftRumble,0);
		}*/
		//Shooter
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


	    heading = gyro->GetAngle();
		//SmartDashboard
	    SmartDashboard::PutNumber("Heading", heading);
		SmartDashboard::PutNumber("Right Speed", shotspeed);
		SmartDashboard::PutNumber("climbspeed",climbspeed);
		SmartDashboard::PutNumber("encRight",Rdis);
		SmartDashboard::PutNumber("encLeft", Ldis);
		SmartDashboard::PutNumber("limitArm", stop_arm1);

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
	const std::string Light = "LIGHT!";

	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)

/* Hardware map of the robot "TBA"  (CB5)
 *	1ft=883.95 ENCODERS
 *
 * 		PWM
 *		0 Front Left
 *		1 	"	Right
 *		2 Back	Left
 *		3	"	Right
 *		4 Kicker
 *		5 Climber
 *		6 Franken Spark
 *		7 Shooter
 *		8
 *		9
 *
 *  RRio Pins
 *  	DIO
 *  	0	A Right Wheel Encoder   (Blue wire)
 *  	1	B "        (Yellow Wire)
 *  	2	A Left Wheel Encoder (Blue Wire)
 *  	3	B "    (Yellow Wire)
 *  	4	A Kicker Encoder
 *  	5	B "
 *  	6 	Limit Switch (kicker arm) for the encoder calibration (registration mark)
 *  	7   Magnetometer Pulse Line (SDC)
 *  	8   Magnetometer Data line  (SDA)
 *  	9
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
 *		1(
 *		2
 *		3(
 *
 *
 *
 *
 *
 *
 */
