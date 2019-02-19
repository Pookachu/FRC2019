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


	//Static lift positon "variables"
	//ball heights
	static constexpr int Lpos1 = 105;
	static constexpr int Lpos2 = 260;
	static constexpr int Lpos3 = 370;
	//panel heights
	static constexpr int Lpos4 = 60;
	static constexpr int Lpos5 = 230;
	static constexpr int Lpos6 = 365;

	static constexpr double hover = 0.05;
	double CurrentLift;

	//Joystick declaration
	frc::Joystick m_stick{0};

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
	//Lift
	frc::PWMVictorSPX m_lift{PWMFour};

	//Tilt
	frc::Spark m_tilt {PWMFive};
	
	//Grab
	frc::Spark m_grab{PWMSix};

	//Servo init
	Servo *m_servo = new Servo(PWMSeven);

	//Sensor definitions
	//Ultrasonic initialization
	//AnalogInput *m_Ultrasonic = new AnalogInput(AnalogZero);

	//Gyroscope initialization
	AHRS  *m_ahrs = new AHRS(SPI::Port::kMXP);

	/*ai = new AnalogInput(0);*/

	//Counter init
	Encoder *m_encoder = new Encoder(0,1, true, Encoder::EncodingType::k4X);

	//Limit switch init
	DigitalInput *m_topLimit = new DigitalInput(3);
	DigitalInput *m_bottomLimit = new DigitalInput(6);
	DigitalInput *m_climbLimit = new DigitalInput(4);

	//Misc Declarations
	//Timer init
 	frc::Timer m_timer;


	//LiveWindow init
	//frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
public:
	//Ultrasonic calculation variables
	//double ultraCal = 3.25;
	//double distToWall;
	//counter variables
//	double diameter = 6/12; // 6 inch wheels
	double dist =0.5*3.14/1024;  // ft per pulse

	bool navxD = 0;
	bool navxDR;

	double target = 0;
	//Robot init function
	bool started = 0;

	bool AdjustUp;
	bool AdjustDown;

	void RobotInit() override
  	{	  	
		//Start the timer
		m_timer.Start();

		//Counter settings
		m_encoder->SetDistancePerPulse(dist);

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
		m_encoder->Reset();
	}
	void AutonomousPeriodic() override
	{
		if(m_timer.Get() <=2)
		{
			if(!(m_bottomLimit->Get()))
			{
				m_lift.Set(0);
			}
			else
			{
			m_encoder->Reset();
			}
		}
		else
		{
			Working();
		}
	}

	void TeleopPeriodic() override
	{
		//Init once (Pragma haha)
		if(started==false)
		{
			started = true;
			//Counter variable declaration
			target = 0;
			m_encoder->Reset();
		}

		Working();

    }

	void Working()
	{
		//DEPRECATED
		/*
		Ultrasonic code
		*/
		//Trying to calculate the ultrasonic sensor value in mm
		//distToWall = (m_Ultrasonic->GetValue()/ultraCal);

		//DISABLED TEMPORARILY
		/*
		Solenoid Control Declaration
		*/
		//m_solenoidOne.Set(m_stick.GetRawButton(bottomTopLeft));
		//m_solenoidTwo.Set(m_stick.GetRawButton(bottomTopRight));

		/*
		Debugging
		*/
		//Joystick HAT testing
		SmartDashboard::PutNumber("POV test",  m_stick.GetPOV());

		//Raw Counter Info
		SmartDashboard::PutNumber("Encoder Ticks", m_encoder->Get());

		//Processed Counter info
		SmartDashboard::PutNumber("Distance", m_encoder->GetDistance());

		//Processed Ultrasonic info
		//SmartDashboard::PutNumber("Distance to wall", distToWall);

		SmartDashboard::PutBoolean("Limit switch top", m_topLimit->Get());
		SmartDashboard::PutBoolean("Limit switch bottom", m_bottomLimit->Get());
		SmartDashboard::PutBoolean("Limit switch climb", m_climbLimit->Get());
		SmartDashboard::PutBoolean("navX drive", navxD);
		SmartDashboard::PutBoolean("navX drive (Really)", navxDR);
		/*
		Seperate Functions for organization and simplicity (Declared below)
		*/

		Tilt();

		Grab();

		//Spin(); //If we need to test the lift with a simple up/down button architecture, re-enable this.

		//SonicCalibration();

		Drive();

		CurrentLift = m_encoder->GetDistance();

		LiftButtonGet();
		LiftMovementLogic();
		LiftManualAdjust();
		
	}

	//DEPRECATED
	//navX Reset Function
	void NavXReset()
	{
		//Gyroscope reset
		bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
		if ( reset_yaw_button_pressed ) 
		{
			m_ahrs->ZeroYaw();
		} 
	}

	void Drive()
	{
		//Assign variables for Joystick axii
		double driveX = m_stick.GetX();
		double driveY = m_stick.GetY();
		double driveZ = m_stick.GetZ();
		//Set deadzone
		double deadZone = 0.1;
		double magnitude = sqrt(driveX * driveX + driveY * driveY);

		driveX /= magnitude;
		driveY /= magnitude;

		if(magnitude < deadZone) 
		{
			magnitude = 0; //no movement in deadzone radius
		} 
		else 
		{
			magnitude -= deadZone; //no discontinuity
		}

		driveX *= magnitude; //scale each amount
		driveY *= magnitude;

		m_drive.DriveCartesian(
		m_stick.GetThrottle() * (driveX*-1), //reversed
		m_stick.GetThrottle() * driveY,
		m_stick.GetThrottle() * -driveZ);
	}

	//DEPRECATED
	//Ultra Sonic Calibration Function Declaration
/*	void SonicCalibration() 
	{	
		if(m_stick.GetRawButton(9)) 
		{
			ultraCal = ultraCal +.01;
		}

		if(m_stick.GetRawButton(7)) 
		{
			ultraCal = ultraCal -.01;
		}
	}*/

	void Grab()
	{
		//Basic control scheme
		if(m_stick.GetRawButton(6))
		{
			m_grab.Set(.5);
		}
		else if(m_stick.GetRawButton(4))
		{
			m_grab.Set(-.5);
		}
		else
		{
			m_grab.Set(0);
		}
	}

	void Tilt()
	{
		//Basic control scheme
		if(m_stick.GetRawButton(5))
		{
			m_tilt.Set(.5);
		}
		else if(m_stick.GetRawButton(3))
		{
			m_tilt.Set(-.5);
		}
		else
		{
			m_tilt.Set(0);
		}
	}

	// Fetch and give LiftTargetControl button pressed info
	void LiftButtonGet()
	{
		
		for(int i = 7; i <= 13; i++)
		{
			//This is a really dumb way to check the 2 button as well as 7-12
			if(i == 13)
			{
				i = 2;
			}
			//Main part of this function, checks buttons 2, and 7-12 to see if they're on and sends if they are to 
			if(CheckButton(i))
			{
				LiftTargetControl(i);
			}
			//Set it back to not screw up the for loop
			if(i == 2)
			{
				i = 13;
			}
		SmartDashboard::PutBoolean("CheckButton",CheckButton(i));
		}
	}

	// Checking to see if a specific button is pressed to be able to implement a for loop
	bool CheckButton(int buttonNumber)
	{
		// Fetching state for button number
		if(m_stick.GetRawButton(buttonNumber))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	// Sets the target variable depending on which button was pressed
	void LiftTargetControl(int button)
	{
		// Assign button value to target
		switch (button)
		{
			case 7:
				target = Lpos3;
				break;
			case 8:
				target = Lpos6;
				break;
			case 9:
				target = Lpos2;
				break;
			case 10:
				target = Lpos5;
				break;
			case 11:
				target = Lpos1;
				break;
			case 12:
				target = Lpos4;
				break;
			default:
			target = 0;
				break;
		}
	}

	//The real control for where the lift is and where it should be
	void LiftMovementLogic()
	{
		// Debug variables
		bool ifHover = false;
		bool ifUp = false;
		bool ifDown = false;
		bool ifBottom = false;

		// Bottom (special case)
		// We want to go to the bottom but are not currently there
		 if (target == 0 && (!(m_bottomLimit->Get() ) ) )
		{
			ifUp = false;
			ifDown = true;
			ifHover = false;
			ifBottom = true;

			m_lift.Set(0);
		}
		// Hover 
		// if within 5 of target value, hover
		else if( (CurrentLift < target+5) && (CurrentLift > target-5) )
			{
				ifUp = false;
				ifDown = false;
				ifHover = true;
				ifBottom = false;

				m_lift.Set(hover);
			}
		// Up
		// If where the lift is is lower than the target, go up
		else if (CurrentLift < target && m_topLimit->Get() ) 
		{
		
			ifUp = true;
			ifDown = false;
			ifHover = false;
			ifBottom = false;
			m_lift.Set(.37);
		}

		// Down
		// If where the lift is is higher than the target, go down
		else if (CurrentLift > target && (! (m_bottomLimit->Get() )))
		{
			ifUp = false;
			ifDown = true;
			ifHover = false;
			ifBottom = false;

			m_lift.Set(-.2);
		} 
		// Encoder reset when we hit the bottom of the lift
		if(m_bottomLimit->Get() == true)
		{
			m_encoder->Reset();
		}
		SmartDashboard::PutNumber("Target:", target);
		SmartDashboard::PutBoolean("Lift Up", ifUp);
		SmartDashboard::PutBoolean("Lift Down", ifDown);
		SmartDashboard::PutBoolean("Lift Hover", ifHover);
		SmartDashboard::PutNumber("Lift Rate", m_encoder->GetRate());
	}

	//Manual adjustments with the POV (Joystick hat)
	void LiftManualAdjust()
	{
		// Target + 5 if POV is up
		if(m_stick.GetPOV() == 0 && AdjustUp == false)
		{
			target = target + 5;
			AdjustUp = true;
			AdjustDown = false;
		}
		// POV middle resets other two to be able to be pressed again
		if(m_stick.GetPOV() == -1)
		{
			AdjustUp = false;
			AdjustDown = false;
		}
		// Target - 5 if POV is down
		if(m_stick.GetPOV() == 180 && AdjustDown == false)
		{
			AdjustUp = false;
			AdjustDown = true;
			target = target - 5;
		}
	}

	//VisionThread Declaration
	static void VisionThread()
	{
	    // Get the USB camera from CameraServer
	    cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

	    // Set the resolution
	    camera.SetResolution(320, 240);

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
