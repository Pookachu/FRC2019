/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <thread>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wpi/raw_ostream.h>
#include <AHRS.h>
#include <AHRSProtocol.h>
#include <frc/WPILib.h>

class Robot : public frc::TimedRobot 
{
#if defined(__linux__)
private:
	//Static PI declaration
	static constexpr double PI = 3.14159265359;

	//Static button declarations
	static constexpr int trigger = 1;
	static constexpr int stickSide = 2;
	static constexpr int topBottomLeft = 3;
	static constexpr int topBottomRight = 4;
	static constexpr int topTopLeft = 5;
	static constexpr int topTopRight = 6;
	static constexpr int bottomTopLeft = 7;
	static constexpr int bottomTopRight = 8;
	static constexpr int bottomMiddleLeft = 9;
	static constexpr int bottomMiddleRight = 10;
	static constexpr int bottomBottomLeft = 11;
	static constexpr int bottomBottomRight = 12;

	//Static PWM port declarations
	static constexpr int PWMZero = 0;
	static constexpr int PWMOne = 1;
	static constexpr int PWMTwo = 2;
	static constexpr int PWMThree = 3;
	static constexpr int PWMFour = 4;
	static constexpr int PWMFive = 5;
	static constexpr int PWMSix = 6;
	static constexpr int PWMSeven = 7;
	static constexpr int PWMEight = 8;
	static constexpr int PWMNine = 9;

	//Static DIO port declarations
	static constexpr int DIOZero = 0;
	static constexpr int DIOOne = 1;
	static constexpr int DIOTwo = 2;
	static constexpr int DIOThree = 3;
	static constexpr int DIOFour = 4;
	static constexpr int DIOFive = 5;
	static constexpr int DIOSix = 6;
	static constexpr int DIOSeven = 7;
	static constexpr int DIOEight = 8;
	static constexpr int DIONine = 9;

	//Static Analog port declarations
	static constexpr int AnalogZero = 0;
	static constexpr int AnalogOne = 1;
	static constexpr double AnalogTwo = 2;
	static constexpr double AnalogThree = 3;

	//Static Relay port declarations
	static constexpr int RelayZero = 0;
	static constexpr int RelayOne = 1;
	static constexpr int RelayTwo = 2;
	static constexpr int RelayThree =3;

	//Static Can port declarations
	static constexpr int CanZero = 0;
	static constexpr int CanOne = 1;
	static constexpr int CanTwo = 2;
	static constexpr int CanThree = 3;

	//Joystick declaration
	frc::Joystick m_stick{1};

	//Drive motor declarations
	/*
    0/| |\2
    1\| |/3
    */
	frc::Spark m_frontLeft{PWMZero};
    frc::Spark m_rearLeft{PWMOne};
    frc::Spark m_frontRight{PWMTwo};
    frc::Spark m_rearRight{PWMThree};

	//Initiating robot drive at m_drive
    frc::MecanumDrive m_drive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};

	//Solenoid declaration (CAN)
	frc::Solenoid m_solenoidOne{CanZero};
	frc::Solenoid m_solenoidTwo{CanOne};

	//Other motor definitions
	frc::PWMVictorSPX m_spinL{PWMFour};
	frc::PWMVictorSPX m_spinR{PWMFive};

	//Ultrasonic initialization
	AnalogInput *m_Ultrasonic = new AnalogInput(AnalogZero);

	//Gyroscope initialization
	AHRS  *m_ahrs = new AHRS(SPI::Port::kMXP);

	//Servo init
	Servo *m_servo = new Servo(1); //Figure out how to wire and get a real port

	/*ai = new AnalogInput(0);*/

	//LiveWindow init
	//frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();

	//Timer init
 	frc::Timer m_timer;

	 //Counter init
	Counter *m_counterMag = new Counter(PWMEight);
	Counter *m_counterIndex = new Counter(PWMNine);

public:
	//Ultrasonic calculation variables
	double ultraCal = 3.25;
	double distToWall;

	//Robot init function
	void RobotInit() override
  	{	  	
		//Start the timer
    	m_timer.Start();

		//Counter settings
		m_counterMag->SetSemiPeriodMode(true);
		m_counterIndex->SetSemiPeriodMode(false);

		/*
		Safeties
		*/
		m_drive.SetSafetyEnabled(false);
		//drive expiration? check later
    	m_drive.SetExpiration(0.1);

	//Start VisionThread in a seperate thread
	#if defined(__linux__)
    	std::thread visionThread(VisionThread);
		visionThread.detach();
	#else
		wpi::errs() << "Vision only available on Linux.\n";
		wpi::errs().flush();
	#endif
	}

	//AUTONOMOUS FUNCTIONS
	void AutonomousInit() override
	{
		//Timer resets before we start autonomous 
		m_timer.Reset();
    	m_timer.Start();

		//Encoder resets before we start autonomous
		m_counterMag->Reset();
		m_counterIndex->Reset();
	}
	void AutonomousPeriodic() override
	{
		
	}

	//TELEOP FUNCTIONS
	void TeleopInit() override
	{

	}

	void TeleopPeriodic() override
	{
		//Counter variable declaration
		double angleDEG;
		double angleRAD;

		//Main While Loop
		while(frc::RobotBase::IsEnabled() && frc::RobotBase::IsOperatorControl())
		{
			/*
			Drive code
			*/
			//Driving with Mecanum (Field oriented with NavX)
			m_drive.DriveCartesian(m_stick.GetX(), (m_stick.GetY()*-1), m_stick.GetZ(), m_ahrs->GetAngle());

			/*
			navX code
			*/
			//Gyroscope reset
			bool reset_yaw_button_pressed = m_stick.GetRawButton(bottomBottomRight);
			if ( reset_yaw_button_pressed ) 
			{
				m_ahrs->ZeroYaw();
			} 

			/*
			Ultrasonic code
			*/
			//Trying to calculate the ultrasonic sensor value in mm
			distToWall = (m_Ultrasonic->GetValue()/ultraCal);

			/*
			Solenoid Control Declaration
			*/
			m_solenoidOne.Set(m_stick.GetRawButton(bottomTopLeft));
			m_solenoidTwo.Set(m_stick.GetRawButton(bottomTopRight));
			
			/*
			Counter code
			*/
			double Mag = m_counterMag->GetPeriod();
			double Index = m_counterIndex->GetPeriod();

			// The 9.73e-4 is the total period of the PWM output on the am-3749
			// The value will then be divided by the period to get duty cycle.
			// This is converted to degrees and Radians
			angleDEG = (Mag/9.739499999999999E-4)*361 -1;
			angleRAD = (Mag/9.739499999999999E-4)*2*(PI);

			/*
			Debugging
			*/
			//Joystick HAT testing
			SmartDashboard::PutNumber("POV test",  m_stick.GetPOV());
			
			SmartDashboard::PutNumber("POV count test",  m_stick.GetPOVCount());

			//Raw Counter Info
			SmartDashboard::PutNumber("Rotations", m_counterIndex->Get());
			SmartDashboard::PutNumber("Intermediate", Mag);

			//Processed Counter info
			SmartDashboard::PutNumber("Angle in Degrees", angleDEG);
			SmartDashboard::PutNumber("Angle in Radians", angleRAD);

			//Processed Ultrasonic info
			SmartDashboard::PutNumber("Distance to wall", distToWall);

			/*
			Seperate Functions for organization and simplicity (Declared below)
			*/

			//Spin(); If we need to test the lift with a simple up/down button architecture, re-enable this.
			SonicCalibration();
			Lift();
		}
    }

	//TEST FUNCTION (for if we ever need to do anything to test specifically I guess)
	void TestPeriodic() override
	{

	}

	//Ultra Sonic Calibration Function Declaration
	void SonicCalibration() 
	{	
		if(m_stick.GetRawButton(topTopRight)) 
		{
			ultraCal = ultraCal +.01;
		}

		if(m_stick.GetRawButton(topBottomRight)) 
		{
			ultraCal = ultraCal -.01;
		}
	}

	//Lift control code (Spin but PID implementation)
	void Lift()
	{
		/*switch (m_stick::GetPOV())
		{
			case:
				break;
		
			default:
				break;
		} */
	}

	//DEPRECATED
	//Lift Motor spinng logic/control declaration
	void Spin() 
	{
		//If bottom button is pressed and top button is not
		if( m_stick.GetRawButton(topBottomLeft) && ( ! ( m_stick.GetRawButton(topTopLeft) ) ) )
		{
			m_spinL.Set(1);
			m_spinR.Set(-1);
		}
		//If top button is pressed and bottom botton is not
		if( m_stick.GetRawButton (topTopLeft) && ( ! ( m_stick.GetRawButton (topBottomLeft) ) ) )
		{
			m_spinL.Set(-1);
			m_spinR.Set(1);
		}
		//If both top and bottom buttons are not pressed or if both top and bottom buttons are pressed
		if( ( ( ! ( m_stick.GetRawButton (topBottomLeft) ) ) && ( ! ( m_stick.GetRawButton (topTopLeft) ) ) ) || ( m_stick.GetRawButton (topTopLeft) && m_stick.GetRawButton (topBottomLeft) ) ) 
		{
			m_spinL.Set(0);
			m_spinR.Set(0);
		}
	}

	//VisionThread Declaration
	static void VisionThread()
	{
	    // Get the USB camera from CameraServer
	    cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

	    // Set the resolution
	    camera.SetResolution(640, 480);

	    // Get a CvSink. This will capture Mats from the Camera
	    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

	    // Setup a CvSource. This will send images back to the Dashboard
	    cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

	    // Mats are very memory expensive. Lets reuse this Mat.
	    cv::Mat mat;

	    while (true) 
		{
	    	// Tell the CvSink to grab a frame from the camera and
	    	// put it
	    	// in the source mat.  If there is an error notify the
	    	// output.
	    	if (cvSink.GrabFrame(mat) == 0) 
			{
				// Send the output the error.
   		 		outputStream.NotifyError(cvSink.GetError());
   	 			// skip the rest of the current iteration
   	 			continue;
   	 		}
    		// Put a rectangle on the image
    		rectangle(mat, cv::Point(100, 100), cv::Point(400, 400), cv::Scalar(255, 255, 255), 5);
    		// Give the output stream a new image to display
    		outputStream.PutFrame(mat);
    	}
	}
	#endif
};
#ifndef RUNNING_FRC_TESTS
int main() 
{
	 return frc::StartRobot<Robot>(); 
}
#endif
