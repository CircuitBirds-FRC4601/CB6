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
#include <opencv2/core/types.hpp>        //all of these might not be necessary


class Robot: public frc::IterativeRobot {
public:
	float lDrive,rDrive;
	Spark *fLeft =new Spark(0);
	Spark *bLeft =new Spark(1);
	Spark *fRight =new Spark(2);
	Spark *bRight =new Spark(3);
	cs::UsbCamera cam= CameraServer::GetInstance()->StartAutomaticCapture();

	Spark *alex =new Spark(7);

	Joystick *leftStick =new Joystick(0);
	Joystick *rightStick =new Joystick(1);
	frc::DigitalOutput *light =new DigitalOutput(0);

	frc::RobotDrive *robotDrive =new frc::RobotBase (fLeft,bLeft,fRight,bRight);

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

		alex->Set(leftStick->GetRawButton(1));

	}
};

START_ROBOT_CLASS(Robot)
